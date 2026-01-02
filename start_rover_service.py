#!/usr/bin/env python3
"""
CLOVER Rover Service Launcher

Starts the unified motor control service that manages Arduino connection
and provides API for HTML, Telegram, and Autonomous Navigation clients.

Usage:
    python start_rover_service.py
    python start_rover_service.py --port 8081 --serial /dev/ttyUSB0
"""

import os
import sys
import argparse
import signal
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from loguru import logger

# Configure logging
logger.remove()
logger.add(
    sys.stderr,
    format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> - <level>{message}</level>",
    level="INFO"
)
logger.add(
    PROJECT_ROOT / "logs" / "rover_service.log",
    rotation="10 MB",
    retention="7 days",
    level="DEBUG"
)


def load_config() -> dict:
    """Load configuration from YAML file if exists"""
    config_path = PROJECT_ROOT / "config" / "rover_config.yaml"
    try:
        import yaml
        with open(config_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except:
        return {}


def find_serial_port() -> str:
    """Auto-detect Arduino serial port"""
    import glob

    # Common Arduino ports
    patterns = [
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/serial/by-id/*Arduino*",
        "/dev/serial/by-id/*CH340*",
        "/dev/serial/by-id/*CP210*"
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            logger.info(f"Found serial port: {ports[0]}")
            return ports[0]

    return "/dev/ttyUSB0"  # Default


def main():
    parser = argparse.ArgumentParser(
        description="CLOVER Rover Service - Unified Motor Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This service provides a unified API for controlling the CLOVER rover.
Multiple clients (HTML interface, Telegram bot, Autonomous Navigation)
can connect simultaneously without serial port conflicts.

API Endpoints:
  POST /connect        - Connect to Arduino
  POST /disconnect     - Disconnect from Arduino
  GET  /status         - Get rover status
  POST /move           - Move rover {vx, vy, wz}
  POST /stop           - Stop motors
  POST /estop          - Emergency stop
  POST /release        - Release e-stop
  POST /mode           - Set mode
  WS   /ws             - WebSocket for real-time control
        """
    )

    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8081, help="HTTP port")
    parser.add_argument("--serial", help="Arduino serial port (auto-detect if not specified)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--auto-connect", action="store_true", help="Auto-connect to Arduino on startup")

    args = parser.parse_args()

    # Load config
    config = load_config()

    # Determine serial port
    serial_port = args.serial
    if not serial_port:
        serial_port = config.get("arduino", {}).get("port")
    if not serial_port:
        serial_port = find_serial_port()

    baudrate = args.baudrate or config.get("arduino", {}).get("baudrate", 115200)

    # Create logs directory
    (PROJECT_ROOT / "logs").mkdir(exist_ok=True)

    # Print banner
    print("""
╔══════════════════════════════════════════════════════════╗
║           🤖 CLOVER Rover Service                        ║
║              Unified Motor Control                       ║
╠══════════════════════════════════════════════════════════╣
║  HTTP API:    http://{host}:{port}                       ║
║  WebSocket:   ws://{host}:{port}/ws                      ║
║  Serial:      {serial}                                   ║
╚══════════════════════════════════════════════════════════╝
    """.format(
        host=args.host if args.host != "0.0.0.0" else "localhost",
        port=args.port,
        serial=serial_port
    ))

    print("Endpoints:")
    print("  GET  /status     - Rover status")
    print("  POST /connect    - Connect to Arduino")
    print("  POST /disconnect - Disconnect")
    print("  POST /move       - Move {vx, vy, wz}")
    print("  POST /stop       - Stop motors")
    print("  POST /estop      - Emergency stop")
    print("  POST /mode       - Set mode")
    print("  WS   /ws         - Real-time control")
    print()
    print("Press Ctrl+C to stop")
    print("-" * 60)

    # Import and start service
    try:
        from src.services.rover_service import RoverService

        service = RoverService(
            host=args.host,
            port=args.port,
            serial_port=serial_port,
            baudrate=baudrate
        )

        # Handle graceful shutdown
        def signal_handler(sig, frame):
            print("\n\n⏹️  Stopping service...")
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Run the service
        service.run()

    except ImportError as e:
        logger.error(f"Import error: {e}")
        print(f"\n❌ Missing dependencies: {e}")
        print("   Install with: pip install aiohttp pymodbus pyserial")
        sys.exit(1)
    except Exception as e:
        logger.exception(f"Service error: {e}")
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
