#!/usr/bin/env python3
"""
CLOVER Unified Teleoperation Server
Combines WebRTC video streaming and WebSocket control for Meta Quest 3

Architecture:
    ┌─────────────────────────────────────────────────────────────┐
    │                    TeleopServer                             │
    │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
    │  │StereoCamera │→ │VideoStream  │→ │ WebRTC (port 8080)  │ │
    │  │ (IMX219 x2) │  │ (H.264)     │  │                     │ │
    │  └─────────────┘  └─────────────┘  └─────────────────────┘ │
    │        ↓                                                    │
    │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
    │  │ Detection   │  │ControlServer│← │ WebSocket (8081)    │ │
    │  │ (optional)  │  │ (joystick)  │  │                     │ │
    │  └─────────────┘  └─────────────┘  └─────────────────────┘ │
    │        ↓                ↓                                   │
    │  ┌─────────────────────────────────────────────────────┐   │
    │  │              Modbus RTU → Arduino                   │   │
    │  └─────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────┘
"""

import asyncio
import signal
import json
from typing import Optional, Callable
from dataclasses import dataclass, asdict
from loguru import logger

from aiohttp import web

import sys
sys.path.insert(0, '/home/jetsonnano/autonomous-rover')

from src.vision.stereo_camera import StereoCamera, StereoFrame
from src.teleop.streaming_server import TeleopStreamingServer, StreamConfig, StereoVideoTrack
from src.teleop.control_server import TeleopControlServer, ControlMode
from src.communication.modbus_client import CloverModbusClient


@dataclass
class TeleopConfig:
    """Unified teleop configuration"""
    # Network
    host: str = "0.0.0.0"
    video_port: int = 8080
    control_port: int = 8081

    # Video
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 30
    video_bitrate: int = 8000000
    stereo_mode: str = "sbs"

    # Control
    max_linear_speed: float = 0.5
    max_angular_speed: float = 1.0
    deadzone: float = 0.1

    # Hardware
    left_camera_id: int = 0
    right_camera_id: int = 1
    modbus_port: str = "/dev/ttyACM0"
    enable_modbus: bool = True


class TeleopServer:
    """
    Unified teleoperation server for CLOVER rover
    Manages stereo camera, video streaming, and joystick control
    """

    def __init__(self, config: Optional[TeleopConfig] = None):
        """
        Initialize unified teleop server

        Args:
            config: TeleopConfig or None for defaults
        """
        self.config = config or TeleopConfig()
        self._running = False

        # Components
        self.stereo_camera: Optional[StereoCamera] = None
        self.streaming_server: Optional[TeleopStreamingServer] = None
        self.control_server: Optional[TeleopControlServer] = None
        self.modbus_client: Optional[CloverModbusClient] = None

        # Callbacks for external consumers (autonomous navigation, etc.)
        self._frame_callbacks: list = []
        self._detection_callback: Optional[Callable] = None

        logger.info("TeleopServer initialized")
        logger.info(f"  Video: {self.config.host}:{self.config.video_port}")
        logger.info(f"  Control: {self.config.host}:{self.config.control_port}")

    def _init_modbus(self) -> bool:
        """Initialize Modbus connection"""
        if not self.config.enable_modbus:
            logger.info("Modbus disabled in config")
            return False

        try:
            self.modbus_client = CloverModbusClient(port=self.config.modbus_port)
            if self.modbus_client.connect():
                logger.success("Modbus connected")
                return True
            else:
                logger.warning("Modbus connection failed")
                return False
        except Exception as e:
            logger.error(f"Modbus init error: {e}")
            return False

    def _init_camera(self) -> bool:
        """Initialize stereo camera"""
        try:
            self.stereo_camera = StereoCamera(
                left_sensor_id=self.config.left_camera_id,
                right_sensor_id=self.config.right_camera_id,
                width=self.config.camera_width,
                height=self.config.camera_height,
                fps=self.config.camera_fps
            )

            # Register frame callback for external consumers
            self.stereo_camera.register_callback(
                "teleop_main",
                self._on_frame
            )

            return True
        except Exception as e:
            logger.error(f"Camera init error: {e}")
            return False

    def _init_streaming(self) -> bool:
        """Initialize video streaming server"""
        try:
            stream_config = StreamConfig(
                host=self.config.host,
                port=self.config.video_port,
                video_width=self.config.camera_width * 2,  # Side-by-side
                video_height=self.config.camera_height,
                video_fps=self.config.camera_fps,
                video_bitrate=self.config.video_bitrate,
                stereo_mode=self.config.stereo_mode
            )

            self.streaming_server = TeleopStreamingServer(
                stereo_camera=self.stereo_camera,
                config=stream_config
            )

            return True
        except Exception as e:
            logger.error(f"Streaming init error: {e}")
            return False

    def _init_control(self) -> bool:
        """Initialize control server"""
        try:
            self.control_server = TeleopControlServer(
                modbus_client=self.modbus_client,
                host=self.config.host,
                port=self.config.control_port,
                max_linear_speed=self.config.max_linear_speed,
                max_angular_speed=self.config.max_angular_speed,
                deadzone=self.config.deadzone
            )

            # Register mode change callback
            self.control_server.on_mode_change(self._on_mode_change)

            return True
        except Exception as e:
            logger.error(f"Control init error: {e}")
            return False

    def _on_frame(self, frame: StereoFrame):
        """Handle new stereo frame"""
        # Forward to registered callbacks (for detection, navigation, etc.)
        for callback in self._frame_callbacks:
            try:
                callback(frame)
            except Exception as e:
                logger.error(f"Frame callback error: {e}")

    def _on_mode_change(self, mode: ControlMode):
        """Handle control mode change"""
        logger.info(f"Control mode: {mode.value}")

        # Notify external systems (e.g., pause autonomous navigation)
        if mode == ControlMode.TELEOP:
            logger.info("Teleop mode active - manual control enabled")
        elif mode == ControlMode.AUTONOMOUS:
            logger.info("Autonomous mode active")
        else:
            logger.info("Control disabled")

    def register_frame_callback(self, callback: Callable[[StereoFrame], None]):
        """
        Register callback for stereo frames

        This allows external systems (detection, navigation) to receive
        the same frames as VR streaming without additional camera access.
        """
        self._frame_callbacks.append(callback)
        logger.info(f"Frame callback registered (total: {len(self._frame_callbacks)})")

    def unregister_frame_callback(self, callback: Callable):
        """Unregister a frame callback"""
        if callback in self._frame_callbacks:
            self._frame_callbacks.remove(callback)

    def get_control_mode(self) -> ControlMode:
        """Get current control mode"""
        if self.control_server:
            return self.control_server.mode
        return ControlMode.DISABLED

    def set_control_mode(self, mode: ControlMode):
        """Set control mode"""
        if self.control_server:
            self.control_server.set_mode(mode)

    async def start(self):
        """Start all teleop services"""
        logger.info("Starting CLOVER Teleop Server...")

        # Initialize components
        self._init_modbus()

        if not self._init_camera():
            logger.error("Failed to initialize camera")
            return False

        if not self._init_streaming():
            logger.error("Failed to initialize streaming")
            return False

        if not self._init_control():
            logger.error("Failed to initialize control")
            return False

        # Start camera capture
        self.stereo_camera.start()

        # Start servers
        await self.streaming_server.start()
        await self.control_server.start()

        self._running = True
        logger.success("=" * 50)
        logger.success("  CLOVER Teleop Server Running")
        logger.success("=" * 50)
        logger.success(f"  Video Stream:  http://{self.config.host}:{self.config.video_port}")
        logger.success(f"  Control WS:    ws://{self.config.host}:{self.config.control_port}/ws")
        logger.success(f"  Status:        http://{self.config.host}:{self.config.control_port}/status")
        logger.success("=" * 50)

        return True

    async def run_forever(self):
        """Run server until interrupted"""
        if not await self.start():
            return

        # Wait for shutdown signal
        stop_event = asyncio.Event()

        def signal_handler():
            logger.info("Shutdown signal received")
            stop_event.set()

        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, signal_handler)

        await stop_event.wait()
        await self.shutdown()

    async def shutdown(self):
        """Shutdown all services gracefully"""
        logger.info("Shutting down teleop server...")
        self._running = False

        # Stop control (stops motors)
        if self.control_server:
            await self.control_server.stop()

        # Stop streaming
        if self.streaming_server:
            await self.streaming_server.shutdown()

        # Stop camera
        if self.stereo_camera:
            self.stereo_camera.release()

        # Disconnect Modbus
        if self.modbus_client:
            self.modbus_client.disconnect()

        logger.success("Teleop server shutdown complete")

    def get_status(self) -> dict:
        """Get overall system status"""
        return {
            "running": self._running,
            "camera": self.stereo_camera.get_properties() if self.stereo_camera else None,
            "modbus_connected": self.modbus_client.is_connected if self.modbus_client else False,
            "control_mode": self.control_server.mode.value if self.control_server else "unknown",
            "connected_clients": len(self.control_server.clients) if self.control_server else 0,
            "config": asdict(self.config)
        }


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description="CLOVER Unified Teleoperation Server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Network options
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--video-port", type=int, default=8080, help="Video streaming port")
    parser.add_argument("--control-port", type=int, default=8081, help="Control WebSocket port")

    # Camera options
    parser.add_argument("--width", type=int, default=1280, help="Camera width")
    parser.add_argument("--height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=int, default=30, help="Framerate")
    parser.add_argument("--left-cam", type=int, default=0, help="Left camera sensor ID")
    parser.add_argument("--right-cam", type=int, default=1, help="Right camera sensor ID")

    # Control options
    parser.add_argument("--max-speed", type=float, default=0.5, help="Max linear speed (m/s)")
    parser.add_argument("--max-rotation", type=float, default=1.0, help="Max angular speed (rad/s)")

    # Hardware options
    parser.add_argument("--modbus-port", default="/dev/ttyACM0", help="Modbus serial port")
    parser.add_argument("--no-modbus", action="store_true", help="Disable Modbus")

    args = parser.parse_args()

    # Create config from args
    config = TeleopConfig(
        host=args.host,
        video_port=args.video_port,
        control_port=args.control_port,
        camera_width=args.width,
        camera_height=args.height,
        camera_fps=args.fps,
        left_camera_id=args.left_cam,
        right_camera_id=args.right_cam,
        max_linear_speed=args.max_speed,
        max_angular_speed=args.max_rotation,
        modbus_port=args.modbus_port,
        enable_modbus=not args.no_modbus
    )

    # Create and run server
    server = TeleopServer(config)

    try:
        asyncio.run(server.run_forever())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")


if __name__ == "__main__":
    main()
