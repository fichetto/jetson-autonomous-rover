#!/usr/bin/env python3
"""
CLOVER Teleoperation Launcher
Quick start script for VR teleoperation with Meta Quest 3
"""

import sys
import argparse
import subprocess
from pathlib import Path

def check_dependencies():
    """Check if required packages are installed"""
    missing = []

    try:
        import aiohttp
    except ImportError:
        missing.append("aiohttp")

    try:
        import aiortc
    except ImportError:
        missing.append("aiortc")

    try:
        import av
    except ImportError:
        missing.append("av")

    if missing:
        print("Missing dependencies:")
        for dep in missing:
            print(f"  - {dep}")
        print("\nInstall with:")
        print("  pip3 install aiohttp aiortc av")
        return False

    return True

def get_local_ip():
    """Get local IP address"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

def main():
    parser = argparse.ArgumentParser(
        description="CLOVER VR Teleoperation Server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./start_teleop.py                    # Start with defaults
  ./start_teleop.py --no-modbus        # Test mode without Arduino
  ./start_teleop.py --width 640 --height 480  # Lower resolution

Connect from Quest 3:
  1. Ensure Quest 3 is on same WiFi network
  2. Configure app with displayed IP address
  3. Put on headset and use controllers to drive!

Controls:
  Left Stick   - Move forward/back, strafe
  Right Stick  - Rotate
  Left Trigger - Slow mode
  Right Trigger - Fast mode
  Button B     - EMERGENCY STOP
        """
    )

    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--video-port", type=int, default=8080, help="Video streaming port")
    parser.add_argument("--control-port", type=int, default=8081, help="Control WebSocket port")
    parser.add_argument("--width", type=int, default=1280, help="Camera width")
    parser.add_argument("--height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=int, default=30, help="Framerate")
    parser.add_argument("--no-modbus", action="store_true", help="Run without Modbus (test mode)")
    parser.add_argument("--max-speed", type=float, default=0.5, help="Max linear speed (m/s)")

    args = parser.parse_args()

    # Check dependencies
    if not check_dependencies():
        sys.exit(1)

    # Get IP for display
    local_ip = get_local_ip()

    print("""
╔══════════════════════════════════════════════════════════════╗
║           🤖 CLOVER VR Teleoperation Server 🎮              ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Configure Meta Quest 3 app with:                           ║
║                                                              ║""")
    print(f"║    📹 Video Server:   http://{local_ip}:{args.video_port}".ljust(62) + "║")
    print(f"║    🎮 Control Server: ws://{local_ip}:{args.control_port}/ws".ljust(62) + "║")
    print("""║                                                              ║
║  Camera: """ + f"{args.width}x{args.height}@{args.fps}fps (stereo)".ljust(43) + """║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")

    # Build command
    cmd = [
        sys.executable,
        "-m", "src.teleop.teleop_server",
        "--host", args.host,
        "--video-port", str(args.video_port),
        "--control-port", str(args.control_port),
        "--width", str(args.width),
        "--height", str(args.height),
        "--fps", str(args.fps),
        "--max-speed", str(args.max_speed),
    ]

    if args.no_modbus:
        cmd.append("--no-modbus")

    # Run server
    try:
        subprocess.run(cmd, cwd=Path(__file__).parent)
    except KeyboardInterrupt:
        print("\n\nShutting down...")

if __name__ == "__main__":
    main()
