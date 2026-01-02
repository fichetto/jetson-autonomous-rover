#!/usr/bin/env python3
"""
CLOVER Rover Service - Unified Motor Control

Central service that manages the single Arduino connection and provides
HTTP/WebSocket API for multiple clients (HTML, Telegram, Autonomous Navigation).

Architecture:
- Single Modbus RTU connection to Arduino
- HTTP endpoints for commands and status
- WebSocket for real-time control and state broadcast
- Watchdog: stops motors if no command received for 2 seconds
- Priority arbitration: e-stop > autonomous > teleop > manual
"""

import asyncio
import json
import time
from typing import Optional, Dict, Any, Set
from dataclasses import dataclass, asdict, field
from enum import Enum
from aiohttp import web, WSMsgType
from loguru import logger

import sys
sys.path.insert(0, '/home/jetsonnano/autonomous-rover')

from src.communication.modbus_client import CloverModbusClient
from src.control.mechanum_drive import MechanumDrive, create_clover_drive


class RoverMode(Enum):
    """Rover operating modes with priority (higher = more priority)"""
    DISABLED = ("disabled", 0)
    MANUAL = ("manual", 1)
    TELEOP = ("teleop", 2)
    AUTONOMOUS = ("autonomous", 3)
    ESTOP = ("estop", 100)  # Highest priority

    def __init__(self, mode_name: str, priority: int):
        self.mode_name = mode_name
        self.priority = priority

    @classmethod
    def from_string(cls, s: str) -> 'RoverMode':
        for mode in cls:
            if mode.mode_name == s.lower():
                return mode
        return cls.DISABLED


@dataclass
class RoverState:
    """Complete rover state - single source of truth"""
    connected: bool = False
    mode: str = "disabled"
    emergency_stop: bool = False
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    motor_speeds: Dict[str, int] = field(default_factory=lambda: {"FL": 0, "FR": 0, "RL": 0, "RR": 0})
    encoder_rpms: Dict[str, float] = field(default_factory=lambda: {"FL": 0, "FR": 0, "RL": 0, "RR": 0})
    velocity: Dict[str, float] = field(default_factory=lambda: {"vx": 0, "vy": 0, "wz": 0})
    last_command_time: float = 0.0
    error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "connected": self.connected,
            "mode": self.mode,
            "emergency_stop": self.emergency_stop,
            "battery_voltage": round(self.battery_voltage, 2),
            "battery_percentage": round(self.battery_percentage, 1),
            "motor_speeds": self.motor_speeds,
            "encoder_rpms": {k: round(v, 1) for k, v in self.encoder_rpms.items()},
            "velocity": {k: round(v, 3) for k, v in self.velocity.items()},
            "error": self.error
        }


class RoverService:
    """
    Unified Rover Control Service

    Manages single Arduino connection and provides API for multiple clients.
    """

    # Battery thresholds (3S LiPo)
    BATTERY_MIN = 9.0
    BATTERY_MAX = 12.6
    BATTERY_WARNING = 10.0

    # Watchdog timeout (seconds)
    WATCHDOG_TIMEOUT = 2.0

    # State broadcast interval (seconds)
    BROADCAST_INTERVAL = 0.1

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8081,
        serial_port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        max_linear_speed: float = 0.5,
        max_angular_speed: float = 1.5
    ):
        self.host = host
        self.port = port
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        # Core components
        self._modbus: Optional[CloverModbusClient] = None
        self._drive = create_clover_drive()

        # State
        self._state = RoverState()
        self._mode = RoverMode.DISABLED
        self._lock = asyncio.Lock()

        # WebSocket clients
        self._ws_clients: Set[web.WebSocketResponse] = set()

        # Background tasks
        self._running = False
        self._watchdog_task: Optional[asyncio.Task] = None
        self._broadcast_task: Optional[asyncio.Task] = None
        self._telemetry_task: Optional[asyncio.Task] = None

        # Web application
        self.app = web.Application()
        self._setup_routes()

        logger.info(f"RoverService initialized on {host}:{port}")

    def _setup_routes(self):
        """Setup HTTP and WebSocket routes"""
        # Connection management
        self.app.router.add_post("/connect", self._handle_connect)
        self.app.router.add_post("/disconnect", self._handle_disconnect)

        # Status
        self.app.router.add_get("/status", self._handle_status)

        # Motor control
        self.app.router.add_post("/move", self._handle_move)
        self.app.router.add_post("/stop", self._handle_stop)
        self.app.router.add_post("/estop", self._handle_estop)
        self.app.router.add_post("/release", self._handle_release)

        # Mode control
        self.app.router.add_post("/mode", self._handle_mode)

        # WebSocket
        self.app.router.add_get("/ws", self._handle_websocket)

        # Health check
        self.app.router.add_get("/health", self._handle_health)

        # CORS middleware
        self.app.middlewares.append(self._cors_middleware)

    @web.middleware
    async def _cors_middleware(self, request, handler):
        """Add CORS headers to all responses"""
        response = await handler(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return response

    # ==================== Connection Management ====================

    async def _handle_connect(self, request: web.Request) -> web.Response:
        """Connect to Arduino"""
        try:
            data = await request.json() if request.body_exists else {}
            port = data.get("port", self.serial_port)
            baudrate = data.get("baudrate", self.baudrate)

            async with self._lock:
                # Disconnect existing connection
                if self._modbus:
                    try:
                        self._modbus.stop_all_motors()
                        self._modbus.disconnect()
                    except:
                        pass

                # Create new connection
                self._modbus = CloverModbusClient(port=port, baudrate=baudrate)
                if self._modbus.connect():
                    self._state.connected = True
                    self._state.error = None
                    self.serial_port = port
                    logger.success(f"Connected to Arduino on {port}")

                    # Start telemetry reading
                    if self._telemetry_task is None or self._telemetry_task.done():
                        self._telemetry_task = asyncio.create_task(self._telemetry_loop())

                    return web.json_response({
                        "success": True,
                        "connected": True,
                        "port": port
                    })
                else:
                    self._modbus = None
                    self._state.connected = False
                    self._state.error = "Connection failed"
                    return web.json_response({
                        "success": False,
                        "error": "Connection failed"
                    }, status=500)

        except Exception as e:
            logger.error(f"Connect error: {e}")
            self._state.error = str(e)
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)

    async def _handle_disconnect(self, request: web.Request) -> web.Response:
        """Disconnect from Arduino"""
        async with self._lock:
            await self._disconnect()

        return web.json_response({
            "success": True,
            "connected": False
        })

    async def _disconnect(self):
        """Internal disconnect method"""
        if self._modbus:
            try:
                self._modbus.stop_all_motors()
                self._modbus.disconnect()
            except:
                pass
            self._modbus = None

        self._state.connected = False
        self._state.motor_speeds = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self._mode = RoverMode.DISABLED
        self._state.mode = "disabled"
        logger.info("Disconnected from Arduino")

    # ==================== Status ====================

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Get current rover status"""
        return web.json_response(self._state.to_dict())

    async def _handle_health(self, request: web.Request) -> web.Response:
        """Health check endpoint"""
        return web.json_response({
            "status": "ok",
            "connected": self._state.connected,
            "mode": self._state.mode
        })

    # ==================== Motor Control ====================

    async def _handle_move(self, request: web.Request) -> web.Response:
        """Move rover with velocity command"""
        if not self._state.connected:
            return web.json_response({"success": False, "error": "Not connected"}, status=400)

        if self._state.emergency_stop:
            return web.json_response({"success": False, "error": "Emergency stop active"}, status=400)

        try:
            data = await request.json()
            vx = float(data.get("vx", 0))
            vy = float(data.get("vy", 0))
            wz = float(data.get("wz", 0))

            # Clamp velocities
            vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
            vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
            wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))

            # Convert to motor speeds
            motor_speeds = self._drive.velocity_to_motor_speeds(vx, vy, wz)

            async with self._lock:
                if self._modbus and self._modbus.is_connected:
                    self._modbus.set_motor_speeds(*motor_speeds)
                    self._state.motor_speeds = {
                        "FL": motor_speeds[0],
                        "FR": motor_speeds[1],
                        "RL": motor_speeds[2],
                        "RR": motor_speeds[3]
                    }
                    self._state.velocity = {"vx": vx, "vy": vy, "wz": wz}
                    self._state.last_command_time = time.time()

            return web.json_response({
                "success": True,
                "motor_speeds": self._state.motor_speeds,
                "velocity": self._state.velocity
            })

        except Exception as e:
            logger.error(f"Move error: {e}")
            return web.json_response({"success": False, "error": str(e)}, status=500)

    async def _handle_stop(self, request: web.Request) -> web.Response:
        """Stop all motors (soft stop)"""
        async with self._lock:
            await self._stop_motors()

        return web.json_response({"success": True, "stopped": True})

    async def _handle_estop(self, request: web.Request) -> web.Response:
        """Emergency stop"""
        async with self._lock:
            await self._emergency_stop()

        return web.json_response({"success": True, "emergency_stop": True})

    async def _handle_release(self, request: web.Request) -> web.Response:
        """Release emergency stop"""
        async with self._lock:
            if self._modbus and self._modbus.is_connected:
                self._modbus.release_emergency_stop()
            self._state.emergency_stop = False
            self._mode = RoverMode.DISABLED
            self._state.mode = "disabled"

        logger.info("Emergency stop released")
        return web.json_response({"success": True, "emergency_stop": False})

    async def _stop_motors(self):
        """Internal stop motors method"""
        if self._modbus and self._modbus.is_connected:
            try:
                self._modbus.stop_all_motors()
            except Exception as e:
                logger.error(f"Stop motors error: {e}")

        self._state.motor_speeds = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self._state.velocity = {"vx": 0, "vy": 0, "wz": 0}

    async def _emergency_stop(self):
        """Internal emergency stop method"""
        logger.warning("EMERGENCY STOP ACTIVATED")
        self._state.emergency_stop = True
        self._mode = RoverMode.ESTOP
        self._state.mode = "estop"

        if self._modbus and self._modbus.is_connected:
            try:
                self._modbus.emergency_stop()
            except Exception as e:
                logger.error(f"E-stop error: {e}")

        self._state.motor_speeds = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self._state.velocity = {"vx": 0, "vy": 0, "wz": 0}

        # Broadcast emergency stop to all clients
        await self._broadcast({
            "type": "emergency_stop",
            "timestamp": time.time()
        })

    # ==================== Mode Control ====================

    async def _handle_mode(self, request: web.Request) -> web.Response:
        """Set rover mode"""
        try:
            data = await request.json()
            mode_str = data.get("mode", "disabled")
            new_mode = RoverMode.from_string(mode_str)

            async with self._lock:
                if self._state.emergency_stop and new_mode != RoverMode.ESTOP:
                    return web.json_response({
                        "success": False,
                        "error": "Release emergency stop first"
                    }, status=400)

                self._mode = new_mode
                self._state.mode = new_mode.mode_name

                # Stop motors when switching to disabled
                if new_mode == RoverMode.DISABLED:
                    await self._stop_motors()

                # Update Arduino mode register
                if self._modbus and self._modbus.is_connected:
                    if new_mode == RoverMode.AUTONOMOUS:
                        self._modbus.set_mode("auto")
                    else:
                        self._modbus.set_mode("manual")

            logger.info(f"Mode changed to: {new_mode.mode_name}")
            return web.json_response({
                "success": True,
                "mode": new_mode.mode_name
            })

        except Exception as e:
            logger.error(f"Mode change error: {e}")
            return web.json_response({"success": False, "error": str(e)}, status=500)

    # ==================== WebSocket ====================

    async def _handle_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """Handle WebSocket connection"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self._ws_clients.add(ws)
        client_id = f"client_{id(ws)}"
        logger.info(f"WebSocket client connected: {client_id}")

        # Send initial state
        await ws.send_json({
            "type": "connected",
            "client_id": client_id,
            "state": self._state.to_dict()
        })

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_ws_message(ws, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._ws_clients.discard(ws)
            logger.info(f"WebSocket client disconnected: {client_id}")

        return ws

    async def _handle_ws_message(self, ws: web.WebSocketResponse, data: str):
        """Process WebSocket message"""
        try:
            msg = json.loads(data)
            msg_type = msg.get("type")

            if msg_type == "move":
                vx = float(msg.get("vx", 0))
                vy = float(msg.get("vy", 0))
                wz = float(msg.get("wz", 0))
                await self._process_move(vx, vy, wz)

            elif msg_type == "controller_state":
                # Quest 3 controller format
                lx = float(msg.get("left_thumbstick_x", 0))
                ly = float(msg.get("left_thumbstick_y", 0))
                rx = float(msg.get("right_thumbstick_x", 0))

                # Apply deadzone
                deadzone = 0.1
                lx = 0 if abs(lx) < deadzone else lx
                ly = 0 if abs(ly) < deadzone else ly
                rx = 0 if abs(rx) < deadzone else rx

                # Convert to velocities
                vx = ly * self.max_linear_speed
                vy = lx * self.max_linear_speed
                wz = -rx * self.max_angular_speed

                await self._process_move(vx, vy, wz)

            elif msg_type == "stop":
                async with self._lock:
                    await self._stop_motors()

            elif msg_type == "estop":
                async with self._lock:
                    await self._emergency_stop()

            elif msg_type == "set_mode":
                mode_str = msg.get("mode", "disabled")
                new_mode = RoverMode.from_string(mode_str)
                async with self._lock:
                    if not self._state.emergency_stop:
                        self._mode = new_mode
                        self._state.mode = new_mode.mode_name

            elif msg_type == "ping":
                await ws.send_json({"type": "pong", "timestamp": time.time()})

        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON: {e}")
        except Exception as e:
            logger.error(f"WS message error: {e}")

    async def _process_move(self, vx: float, vy: float, wz: float):
        """Process movement command"""
        if not self._state.connected or self._state.emergency_stop:
            return

        # Clamp velocities
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))

        # Convert to motor speeds
        motor_speeds = self._drive.velocity_to_motor_speeds(vx, vy, wz)

        async with self._lock:
            if self._modbus and self._modbus.is_connected:
                try:
                    self._modbus.set_motor_speeds(*motor_speeds)
                    self._state.motor_speeds = {
                        "FL": motor_speeds[0],
                        "FR": motor_speeds[1],
                        "RL": motor_speeds[2],
                        "RR": motor_speeds[3]
                    }
                    self._state.velocity = {"vx": vx, "vy": vy, "wz": wz}
                    self._state.last_command_time = time.time()
                except Exception as e:
                    logger.error(f"Motor command error: {e}")

    async def _broadcast(self, message: dict):
        """Broadcast message to all WebSocket clients"""
        if not self._ws_clients:
            return

        for ws in list(self._ws_clients):
            try:
                await ws.send_json(message)
            except Exception as e:
                logger.error(f"Broadcast error: {e}")
                self._ws_clients.discard(ws)

    # ==================== Background Tasks ====================

    async def _watchdog_loop(self):
        """Watchdog: stop motors if no command for WATCHDOG_TIMEOUT seconds"""
        while self._running:
            await asyncio.sleep(0.5)

            if not self._state.connected:
                continue

            if self._mode in [RoverMode.TELEOP, RoverMode.MANUAL]:
                elapsed = time.time() - self._state.last_command_time
                if elapsed > self.WATCHDOG_TIMEOUT and any(self._state.motor_speeds.values()):
                    logger.warning(f"Watchdog timeout ({elapsed:.1f}s) - stopping motors")
                    async with self._lock:
                        await self._stop_motors()

    async def _broadcast_loop(self):
        """Periodically broadcast state to all clients"""
        while self._running:
            await asyncio.sleep(self.BROADCAST_INTERVAL)

            if self._ws_clients:
                await self._broadcast({
                    "type": "state",
                    **self._state.to_dict()
                })

    async def _telemetry_loop(self):
        """Read telemetry from Arduino"""
        while self._running and self._state.connected:
            await asyncio.sleep(1.0)  # Read every second

            if not self._modbus or not self._modbus.is_connected:
                continue

            try:
                async with self._lock:
                    # Read battery voltage
                    voltage = self._modbus.read_battery_voltage()
                    if voltage:
                        self._state.battery_voltage = voltage
                        self._state.battery_percentage = self._calc_battery_percentage(voltage)

                    # Read encoder speeds
                    encoder_speeds = self._modbus.read_encoder_speeds()
                    if encoder_speeds:
                        self._state.encoder_rpms = encoder_speeds

            except Exception as e:
                logger.error(f"Telemetry error: {e}")

    def _calc_battery_percentage(self, voltage: float) -> float:
        """Calculate battery percentage from voltage"""
        if voltage <= self.BATTERY_MIN:
            return 0.0
        elif voltage >= self.BATTERY_MAX:
            return 100.0
        else:
            return ((voltage - self.BATTERY_MIN) / (self.BATTERY_MAX - self.BATTERY_MIN)) * 100.0

    # ==================== Lifecycle ====================

    async def start(self):
        """Start the service"""
        self._running = True

        # Start background tasks
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())

        logger.success(f"RoverService started on http://{self.host}:{self.port}")

    async def stop(self):
        """Stop the service"""
        self._running = False

        # Cancel background tasks
        for task in [self._watchdog_task, self._broadcast_task, self._telemetry_task]:
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        # Stop motors and disconnect
        await self._stop_motors()
        await self._disconnect()

        logger.info("RoverService stopped")

    def run(self):
        """Run the service (blocking)"""
        async def runner():
            await self.start()
            runner = web.AppRunner(self.app)
            await runner.setup()
            site = web.TCPSite(runner, self.host, self.port)
            await site.start()

            # Keep running
            try:
                while True:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                pass
            finally:
                await self.stop()
                await runner.cleanup()

        try:
            asyncio.run(runner())
        except KeyboardInterrupt:
            logger.info("Shutting down...")


# Standalone execution
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CLOVER Rover Service")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8081, help="HTTP port")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Arduino serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    args = parser.parse_args()

    service = RoverService(
        host=args.host,
        port=args.port,
        serial_port=args.serial,
        baudrate=args.baudrate
    )
    service.run()
