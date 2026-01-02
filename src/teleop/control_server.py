#!/usr/bin/env python3
"""
WebSocket Control Server for Meta Quest 3 Joystick Input
Receives controller commands and drives CLOVER rover
"""

import asyncio
import json
import time
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass, asdict
from enum import Enum
from loguru import logger

from aiohttp import web, WSMsgType

import sys
sys.path.insert(0, '/home/jetsonnano/autonomous-rover')
from src.control.mechanum_drive import MechanumDrive, create_clover_drive
from src.communication.modbus_client import CloverModbusClient


class ControlMode(Enum):
    """Control mode for the rover"""
    DISABLED = "disabled"           # No control (autonomous or stopped)
    TELEOP = "teleop"               # VR joystick control
    AUTONOMOUS = "autonomous"       # Autonomous navigation


@dataclass
class ControllerState:
    """State of Quest 3 controllers"""
    # Left controller (typically movement)
    left_thumbstick_x: float = 0.0      # -1 to 1 (strafe)
    left_thumbstick_y: float = 0.0      # -1 to 1 (forward/back)
    left_trigger: float = 0.0           # 0 to 1
    left_grip: float = 0.0              # 0 to 1
    left_button_x: bool = False
    left_button_y: bool = False
    left_thumbstick_click: bool = False

    # Right controller (typically rotation/actions)
    right_thumbstick_x: float = 0.0     # -1 to 1 (rotation)
    right_thumbstick_y: float = 0.0     # -1 to 1 (optional)
    right_trigger: float = 0.0          # 0 to 1
    right_grip: float = 0.0             # 0 to 1
    right_button_a: bool = False
    right_button_b: bool = False
    right_thumbstick_click: bool = False

    # Head tracking (optional, for camera control)
    head_pitch: float = 0.0             # degrees
    head_yaw: float = 0.0               # degrees
    head_roll: float = 0.0              # degrees

    timestamp: float = 0.0


@dataclass
class RoverState:
    """Current state of the rover"""
    mode: str = "disabled"
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    motor_speeds: tuple = (0, 0, 0, 0)
    battery_voltage: float = 0.0
    connected: bool = False
    timestamp: float = 0.0


class TeleopControlServer:
    """
    WebSocket server for Quest 3 controller input
    Translates joystick commands to rover motion
    """

    def __init__(
        self,
        modbus_client: Optional[CloverModbusClient] = None,
        host: str = "0.0.0.0",
        port: int = 8081,
        max_linear_speed: float = 0.5,      # m/s
        max_angular_speed: float = 1.0,      # rad/s
        deadzone: float = 0.1                # Joystick deadzone
    ):
        """
        Initialize control server

        Args:
            modbus_client: CloverModbusClient for motor control (optional for testing)
            host: Server bind address
            port: WebSocket port
            max_linear_speed: Maximum linear velocity
            max_angular_speed: Maximum angular velocity
            deadzone: Joystick deadzone threshold
        """
        self.modbus_client = modbus_client
        self.host = host
        self.port = port
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.deadzone = deadzone

        self.app = web.Application()
        self.drive = create_clover_drive()

        # State
        self.mode = ControlMode.DISABLED
        self.controller_state = ControllerState()
        self.rover_state = RoverState()
        self.clients: Dict[str, web.WebSocketResponse] = {}

        # Control loop
        self._running = False
        self._control_task: Optional[asyncio.Task] = None
        self._command_rate = 20  # Hz

        # Callbacks for state changes
        self._on_mode_change: Optional[Callable[[ControlMode], None]] = None
        self._on_command: Optional[Callable[[float, float, float], None]] = None

        self._setup_routes()
        logger.info(f"TeleopControlServer initialized on {host}:{port}")

    def _setup_routes(self):
        """Setup HTTP and WebSocket routes"""
        self.app.router.add_get("/ws", self._handle_websocket)
        self.app.router.add_get("/status", self._handle_status)
        self.app.router.add_post("/mode", self._handle_set_mode)
        self.app.router.add_post("/stop", self._handle_emergency_stop)
        self.app.router.add_post("/arduino/connect", self._handle_arduino_connect)
        self.app.router.add_post("/arduino/disconnect", self._handle_arduino_disconnect)
        self.app.router.add_get("/arduino/status", self._handle_arduino_status)

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        # Rescale value outside deadzone to full range
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _process_controller_input(self, state: ControllerState) -> tuple:
        """
        Convert controller state to rover velocity commands

        Default mapping:
        - Left stick Y: Forward/Backward
        - Left stick X: Strafe Left/Right
        - Right stick X: Rotate

        Returns:
            Tuple of (vx, vy, wz)
        """
        # Apply deadzone
        fwd = self._apply_deadzone(state.left_thumbstick_y)
        strafe = self._apply_deadzone(state.left_thumbstick_x)
        rotate = self._apply_deadzone(state.right_thumbstick_x)

        # Speed boost with triggers
        speed_multiplier = 1.0
        if state.left_trigger > 0.5:
            speed_multiplier = 0.5  # Slow mode (precision)
        elif state.right_trigger > 0.5:
            speed_multiplier = 1.5  # Fast mode (limited to max)

        # Calculate velocities
        vx = fwd * self.max_linear_speed * speed_multiplier
        vy = strafe * self.max_linear_speed * speed_multiplier
        wz = -rotate * self.max_angular_speed * speed_multiplier  # Negative for intuitive rotation

        # Clamp to limits
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))

        return vx, vy, wz

    async def _handle_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """Handle WebSocket connection for controller input"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = f"quest_{len(self.clients)}_{int(time.time())}"
        self.clients[client_id] = ws
        logger.info(f"Quest controller connected: {client_id}")

        # Send initial state
        await ws.send_json({
            "type": "connected",
            "client_id": client_id,
            "mode": self.mode.value,
            "rover_state": asdict(self.rover_state)
        })

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_message(client_id, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            del self.clients[client_id]
            logger.info(f"Quest controller disconnected: {client_id}")

        return ws

    async def _handle_message(self, client_id: str, data: str):
        """Process incoming WebSocket message"""
        try:
            msg = json.loads(data)
            msg_type = msg.get("type")

            if msg_type == "controller_state":
                # Update controller state
                self.controller_state = ControllerState(
                    left_thumbstick_x=msg.get("left_thumbstick_x", 0),
                    left_thumbstick_y=msg.get("left_thumbstick_y", 0),
                    left_trigger=msg.get("left_trigger", 0),
                    left_grip=msg.get("left_grip", 0),
                    left_button_x=msg.get("left_button_x", False),
                    left_button_y=msg.get("left_button_y", False),
                    left_thumbstick_click=msg.get("left_thumbstick_click", False),
                    right_thumbstick_x=msg.get("right_thumbstick_x", 0),
                    right_thumbstick_y=msg.get("right_thumbstick_y", 0),
                    right_trigger=msg.get("right_trigger", 0),
                    right_grip=msg.get("right_grip", 0),
                    right_button_a=msg.get("right_button_a", False),
                    right_button_b=msg.get("right_button_b", False),
                    right_thumbstick_click=msg.get("right_thumbstick_click", False),
                    head_pitch=msg.get("head_pitch", 0),
                    head_yaw=msg.get("head_yaw", 0),
                    head_roll=msg.get("head_roll", 0),
                    timestamp=time.time()
                )

                # Handle special buttons
                if msg.get("right_button_b"):  # Emergency stop
                    await self._emergency_stop()

                if msg.get("left_button_y"):  # Toggle mode
                    new_mode = ControlMode.TELEOP if self.mode == ControlMode.DISABLED else ControlMode.DISABLED
                    self.set_mode(new_mode)

            elif msg_type == "set_mode":
                mode_str = msg.get("mode", "disabled")
                self.set_mode(ControlMode(mode_str))

            elif msg_type == "ping":
                await self._send_to_client(client_id, {"type": "pong", "timestamp": time.time()})

        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON from {client_id}: {e}")
        except Exception as e:
            logger.error(f"Error processing message from {client_id}: {e}")

    async def _send_to_client(self, client_id: str, data: dict):
        """Send message to specific client"""
        if client_id in self.clients:
            await self.clients[client_id].send_json(data)

    async def _broadcast(self, data: dict):
        """Broadcast message to all connected clients"""
        for ws in self.clients.values():
            try:
                await ws.send_json(data)
            except Exception as e:
                logger.error(f"Broadcast error: {e}")

    def set_mode(self, mode: ControlMode):
        """Set control mode"""
        if mode != self.mode:
            old_mode = self.mode
            self.mode = mode
            self.rover_state.mode = mode.value
            logger.info(f"Control mode changed: {old_mode.value} -> {mode.value}")

            if self._on_mode_change:
                self._on_mode_change(mode)

            # Stop rover when disabling
            if mode == ControlMode.DISABLED:
                asyncio.create_task(self._stop_rover())

    async def _emergency_stop(self):
        """Emergency stop - immediately halt all motors"""
        logger.warning("EMERGENCY STOP triggered!")
        self.set_mode(ControlMode.DISABLED)
        await self._stop_rover()

        # Broadcast stop to all clients
        await self._broadcast({
            "type": "emergency_stop",
            "timestamp": time.time()
        })

    async def _stop_rover(self):
        """Stop all motors"""
        if self.modbus_client and self.modbus_client.is_connected:
            try:
                self.modbus_client.emergency_stop()
            except Exception as e:
                logger.error(f"Error stopping motors: {e}")

        self.rover_state.vx = 0
        self.rover_state.vy = 0
        self.rover_state.wz = 0
        self.rover_state.motor_speeds = (0, 0, 0, 0)

    async def _control_loop(self):
        """Main control loop - sends motor commands at fixed rate"""
        interval = 1.0 / self._command_rate

        while self._running:
            start = time.time()

            if self.mode == ControlMode.TELEOP:
                # Process controller input
                vx, vy, wz = self._process_controller_input(self.controller_state)

                # Calculate motor speeds
                motor_speeds = self.drive.velocity_to_motor_speeds(vx, vy, wz)

                # Send to motors
                if self.modbus_client and self.modbus_client.is_connected:
                    try:
                        self.modbus_client.set_motor_speeds(*motor_speeds)
                        self.rover_state.connected = True
                    except Exception as e:
                        logger.error(f"Modbus error: {e}")
                        self.rover_state.connected = False

                # Update state
                self.rover_state.vx = vx
                self.rover_state.vy = vy
                self.rover_state.wz = wz
                self.rover_state.motor_speeds = motor_speeds
                self.rover_state.timestamp = time.time()

                # Callback
                if self._on_command:
                    self._on_command(vx, vy, wz)

                # Send state update to clients
                await self._broadcast({
                    "type": "rover_state",
                    **asdict(self.rover_state)
                })

            # Maintain loop rate
            elapsed = time.time() - start
            sleep_time = max(0, interval - elapsed)
            await asyncio.sleep(sleep_time)

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Get current status"""
        return web.json_response({
            "mode": self.mode.value,
            "rover_state": asdict(self.rover_state),
            "controller_state": asdict(self.controller_state),
            "connected_clients": len(self.clients),
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed
        })

    async def _handle_set_mode(self, request: web.Request) -> web.Response:
        """Set control mode via HTTP"""
        try:
            data = await request.json()
            mode_str = data.get("mode", "disabled")
            self.set_mode(ControlMode(mode_str))
            return web.json_response({"success": True, "mode": self.mode.value})
        except Exception as e:
            return web.json_response({"success": False, "error": str(e)}, status=400)

    async def _handle_emergency_stop(self, request: web.Request) -> web.Response:
        """Emergency stop via HTTP"""
        await self._emergency_stop()
        return web.json_response({"success": True, "stopped": True})

    async def _handle_arduino_connect(self, request: web.Request) -> web.Response:
        """Connect to Arduino via Modbus"""
        try:
            data = await request.json() if request.body_exists else {}
            port = data.get("port", "/dev/ttyUSB0")
            baudrate = data.get("baudrate", 115200)

            # Disconnect existing connection first
            if self.modbus_client:
                try:
                    self.modbus_client.disconnect()
                except:
                    pass

            # Create new connection
            self.modbus_client = CloverModbusClient(port=port, baudrate=baudrate)
            if self.modbus_client.connect():
                self.rover_state.connected = True
                logger.success(f"Arduino connected on {port}")
                return web.json_response({
                    "success": True,
                    "connected": True,
                    "port": port
                })
            else:
                self.modbus_client = None
                self.rover_state.connected = False
                return web.json_response({
                    "success": False,
                    "error": "Connection failed"
                }, status=500)
        except Exception as e:
            logger.error(f"Arduino connect error: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)

    async def _handle_arduino_disconnect(self, request: web.Request) -> web.Response:
        """Disconnect from Arduino"""
        try:
            if self.modbus_client:
                # Stop motors first
                try:
                    self.modbus_client.stop_all_motors()
                except:
                    pass
                self.modbus_client.disconnect()
                self.modbus_client = None

            self.rover_state.connected = False
            self.set_mode(ControlMode.DISABLED)
            logger.info("Arduino disconnected")
            return web.json_response({
                "success": True,
                "connected": False
            })
        except Exception as e:
            logger.error(f"Arduino disconnect error: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)

    async def _handle_arduino_status(self, request: web.Request) -> web.Response:
        """Get Arduino connection status"""
        connected = self.modbus_client is not None and self.modbus_client.is_connected
        status = {
            "connected": connected,
            "port": self.modbus_client.port if self.modbus_client else None
        }

        # Get battery voltage if connected
        if connected:
            try:
                status["battery_voltage"] = self.modbus_client.read_battery_voltage()
            except:
                status["battery_voltage"] = None

        return web.json_response(status)

    async def start(self):
        """Start the control server"""
        self._running = True
        self._control_task = asyncio.create_task(self._control_loop())

        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        logger.success(f"Control server running on ws://{self.host}:{self.port}/ws")

    async def stop(self):
        """Stop the control server"""
        self._running = False
        if self._control_task:
            self._control_task.cancel()
        await self._stop_rover()
        logger.info("Control server stopped")

    def run(self):
        """Run server (blocking)"""
        web.run_app(self.app, host=self.host, port=self.port)

    # Callbacks
    def on_mode_change(self, callback: Callable[[ControlMode], None]):
        """Register callback for mode changes"""
        self._on_mode_change = callback

    def on_command(self, callback: Callable[[float, float, float], None]):
        """Register callback for velocity commands (for autonomous system)"""
        self._on_command = callback


# Standalone test
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CLOVER Control Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8081, help="WebSocket port")
    parser.add_argument("--no-modbus", action="store_true", help="Run without Modbus")
    args = parser.parse_args()

    # Create Modbus client (optional)
    modbus = None
    if not args.no_modbus:
        try:
            modbus = CloverModbusClient()
            if not modbus.connect():
                logger.warning("Modbus connection failed, running in simulation mode")
                modbus = None
        except Exception as e:
            logger.warning(f"Modbus init failed: {e}")
            modbus = None

    # Create and run server
    server = TeleopControlServer(
        modbus_client=modbus,
        host=args.host,
        port=args.port
    )

    try:
        server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        if modbus:
            modbus.disconnect()
