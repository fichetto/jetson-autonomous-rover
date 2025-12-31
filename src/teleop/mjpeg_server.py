#!/usr/bin/env python3
"""
Simple MJPEG Streaming Server for CLOVER
Works with any browser, no WebRTC required
"""

import asyncio
import cv2
import time
from aiohttp import web
from loguru import logger

import sys
sys.path.insert(0, '/home/jetsonnano/autonomous-rover')


class MJPEGServer:
    """
    Simple MJPEG streaming server
    Compatible with any browser and VLC
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8080,
                 width: int = 1280, height: int = 720, fps: int = 30,
                 quality: int = 80):
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality

        self.app = web.Application()
        self.cameras = []
        self.running = False

        self._setup_routes()

    def _setup_routes(self):
        self.app.router.add_get("/", self._handle_index)
        self.app.router.add_get("/stream", self._handle_stream)
        self.app.router.add_get("/stream/left", self._handle_stream_left)
        self.app.router.add_get("/stream/right", self._handle_stream_right)
        self.app.router.add_get("/snapshot", self._handle_snapshot)
        self.app.router.add_get("/health", self._handle_health)
        self.app.router.add_static("/static",
            "/home/jetsonnano/autonomous-rover/src/teleop/static")

    def _gstreamer_pipeline(self, sensor_id: int) -> str:
        """GStreamer pipeline for CSI camera"""
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, "
            f"framerate={self.fps}/1, format=NV12 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=1 max-buffers=2"
        )

    def open_cameras(self) -> bool:
        """Open stereo cameras"""
        try:
            # Try CSI cameras first
            for sensor_id in [0, 1]:
                pipeline = self._gstreamer_pipeline(sensor_id)
                logger.info(f"Opening camera {sensor_id}...")
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

                if not cap.isOpened():
                    # Fallback to V4L2
                    logger.warning(f"CSI camera {sensor_id} failed, trying V4L2")
                    cap = cv2.VideoCapture(sensor_id, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)

                if cap.isOpened():
                    self.cameras.append(cap)
                    logger.success(f"Camera {sensor_id} opened")
                else:
                    logger.error(f"Failed to open camera {sensor_id}")

            return len(self.cameras) > 0

        except Exception as e:
            logger.error(f"Camera error: {e}")
            return False

    def read_stereo_frame(self):
        """Read and combine stereo frames"""
        frames = []
        for cap in self.cameras:
            ret, frame = cap.read()
            if ret:
                # Flip 180 degrees (cameras are upside down)
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                frames.append(frame)
            else:
                # Return black frame if camera fails
                frames.append(np.zeros((self.height, self.width, 3), dtype=np.uint8))

        if len(frames) == 2:
            # Side-by-side stereo
            return cv2.hconcat(frames)
        elif len(frames) == 1:
            return frames[0]
        else:
            return None

    def encode_frame(self, frame, quality=None):
        """Encode frame as JPEG"""
        if quality is None:
            quality = self.quality
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        return buffer.tobytes()

    async def _handle_index(self, request):
        """Serve index page"""
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>CLOVER Teleop</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <style>
        * {{ box-sizing: border-box; margin: 0; padding: 0; }}
        body {{
            background: #1a1a2e;
            color: #eee;
            font-family: system-ui;
            min-height: 100vh;
            overflow-x: hidden;
        }}
        .header {{
            background: #16213e;
            padding: 10px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }}
        .header h1 {{ color: #00d4ff; font-size: 1.2em; }}
        .status {{
            display: flex;
            gap: 10px;
            align-items: center;
        }}
        .status-dot {{
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #ff4757;
        }}
        .status-dot.connected {{ background: #2ed573; }}
        .main-container {{
            display: flex;
            flex-direction: column;
            padding: 10px;
            gap: 10px;
        }}
        .stream-container {{
            flex: 1;
            display: flex;
            justify-content: center;
        }}
        #stream {{
            max-width: 100%;
            max-height: 50vh;
            border-radius: 8px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
        }}
        .control-panel {{
            display: flex;
            gap: 20px;
            justify-content: center;
            flex-wrap: wrap;
            padding: 10px;
        }}
        .drive-controls {{
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 5px;
        }}
        .drive-row {{
            display: flex;
            gap: 5px;
        }}
        .drive-btn {{
            width: 70px;
            height: 70px;
            font-size: 24px;
            background: #16213e;
            color: #00d4ff;
            border: 2px solid #00d4ff;
            border-radius: 12px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            user-select: none;
            touch-action: manipulation;
            transition: all 0.1s;
        }}
        .drive-btn:hover {{ background: #1e3a5f; }}
        .drive-btn:active, .drive-btn.active {{
            background: #00d4ff;
            color: #1a1a2e;
            transform: scale(0.95);
        }}
        .drive-btn.stop {{
            background: #ff4757;
            border-color: #ff4757;
            color: white;
            font-size: 14px;
            font-weight: bold;
        }}
        .drive-btn.stop:active {{ background: #ff3344; }}
        .speed-control {{
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
            padding: 15px;
            background: #16213e;
            border-radius: 12px;
        }}
        .speed-control label {{ font-size: 14px; color: #888; }}
        .speed-control input[type="range"] {{
            width: 150px;
            accent-color: #00d4ff;
        }}
        .speed-value {{
            font-size: 24px;
            font-weight: bold;
            color: #00d4ff;
        }}
        .view-controls {{
            display: flex;
            gap: 5px;
            justify-content: center;
            flex-wrap: wrap;
        }}
        .view-btn {{
            background: #16213e;
            color: #eee;
            border: 1px solid #333;
            padding: 8px 15px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 12px;
        }}
        .view-btn:hover {{ background: #1e3a5f; }}
        .keyboard-hint {{
            text-align: center;
            color: #666;
            font-size: 12px;
            padding: 10px;
        }}
        .keyboard-hint kbd {{
            background: #333;
            padding: 2px 6px;
            border-radius: 4px;
            margin: 0 2px;
        }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 CLOVER Teleop</h1>
        <div class="status">
            <div class="status-dot" id="statusDot"></div>
            <span id="statusText">Disconnesso</span>
        </div>
    </div>

    <div class="main-container">
        <div class="stream-container">
            <img id="stream" src="/stream" alt="CLOVER Camera">
        </div>

        <div class="control-panel">
            <div class="drive-controls">
                <div class="drive-row">
                    <div class="drive-btn" style="visibility:hidden"></div>
                    <button class="drive-btn" id="btnFwd" data-cmd="forward">▲</button>
                    <div class="drive-btn" style="visibility:hidden"></div>
                </div>
                <div class="drive-row">
                    <button class="drive-btn" id="btnLeft" data-cmd="left">◀</button>
                    <button class="drive-btn stop" id="btnStop" data-cmd="stop">STOP</button>
                    <button class="drive-btn" id="btnRight" data-cmd="right">▶</button>
                </div>
                <div class="drive-row">
                    <button class="drive-btn" id="btnRotL" data-cmd="rotL">↺</button>
                    <button class="drive-btn" id="btnBack" data-cmd="backward">▼</button>
                    <button class="drive-btn" id="btnRotR" data-cmd="rotR">↻</button>
                </div>
            </div>

            <div class="speed-control">
                <label>Velocità</label>
                <input type="range" id="speedSlider" min="10" max="100" value="50">
                <div class="speed-value"><span id="speedValue">50</span>%</div>
            </div>
        </div>

        <div class="view-controls">
            <button class="view-btn" onclick="setStream('/stream')">Stereo</button>
            <button class="view-btn" onclick="setStream('/stream/left')">Left</button>
            <button class="view-btn" onclick="setStream('/stream/right')">Right</button>
            <button class="view-btn" onclick="toggleFullscreen()">Fullscreen</button>
        </div>

        <div class="keyboard-hint">
            Tastiera: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> movimento,
            <kbd>Q</kbd><kbd>E</kbd> rotazione,
            <kbd>Spazio</kbd> stop
        </div>
    </div>

    <script>
        // WebSocket connection
        let ws = null;
        let connected = false;
        let speed = 50;
        let activeKeys = new Set();
        let sendInterval = null;

        // Connect to control server
        function connect() {{
            const wsUrl = 'ws://' + window.location.hostname + ':8081/ws';
            console.log('Connecting to', wsUrl);

            ws = new WebSocket(wsUrl);

            ws.onopen = () => {{
                connected = true;
                document.getElementById('statusDot').classList.add('connected');
                document.getElementById('statusText').textContent = 'Connesso';
                console.log('Connected!');
                // Enable teleop mode
                ws.send(JSON.stringify({{type: 'set_mode', mode: 'teleop'}}));
            }};

            ws.onclose = () => {{
                connected = false;
                document.getElementById('statusDot').classList.remove('connected');
                document.getElementById('statusText').textContent = 'Disconnesso';
                console.log('Disconnected');
                // Retry connection
                setTimeout(connect, 2000);
            }};

            ws.onerror = (err) => {{
                console.error('WebSocket error:', err);
            }};

            ws.onmessage = (event) => {{
                const data = JSON.parse(event.data);
                if (data.type === 'rover_state') {{
                    // Could update UI with rover state
                }}
            }};
        }}

        // Send drive command
        function sendCommand(vx, vy, wz) {{
            if (!connected || !ws) return;

            const scale = speed / 100;
            ws.send(JSON.stringify({{
                type: 'controller_state',
                left_thumbstick_x: vy * scale,
                left_thumbstick_y: vx * scale,
                right_thumbstick_x: wz * scale,
                left_trigger: 0,
                right_trigger: 0
            }}));
        }}

        // Calculate command from active keys
        function updateFromKeys() {{
            let vx = 0, vy = 0, wz = 0;

            if (activeKeys.has('w') || activeKeys.has('arrowup')) vx = 1;
            if (activeKeys.has('s') || activeKeys.has('arrowdown')) vx = -1;
            if (activeKeys.has('a') || activeKeys.has('arrowleft')) vy = 1;
            if (activeKeys.has('d') || activeKeys.has('arrowright')) vy = -1;
            if (activeKeys.has('q')) wz = -1;
            if (activeKeys.has('e')) wz = 1;

            sendCommand(vx, vy, wz);
        }}

        // Keyboard controls
        document.addEventListener('keydown', (e) => {{
            const key = e.key.toLowerCase();
            if (['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright',' '].includes(key)) {{
                e.preventDefault();
                if (key === ' ') {{
                    sendCommand(0, 0, 0);
                    activeKeys.clear();
                }} else {{
                    activeKeys.add(key);
                    updateFromKeys();
                }}
            }}
        }});

        document.addEventListener('keyup', (e) => {{
            const key = e.key.toLowerCase();
            activeKeys.delete(key);
            if (activeKeys.size === 0) {{
                sendCommand(0, 0, 0);
            }} else {{
                updateFromKeys();
            }}
        }});

        // Button controls (mouse and touch)
        const btnCommands = {{
            'forward': [1, 0, 0],
            'backward': [-1, 0, 0],
            'left': [0, 1, 0],
            'right': [0, -1, 0],
            'rotL': [0, 0, -1],
            'rotR': [0, 0, 1],
            'stop': [0, 0, 0]
        }};

        document.querySelectorAll('.drive-btn[data-cmd]').forEach(btn => {{
            const cmd = btn.dataset.cmd;
            const [vx, vy, wz] = btnCommands[cmd];

            const startDrive = (e) => {{
                e.preventDefault();
                btn.classList.add('active');
                sendCommand(vx, vy, wz);
            }};

            const stopDrive = (e) => {{
                e.preventDefault();
                btn.classList.remove('active');
                if (cmd !== 'stop') sendCommand(0, 0, 0);
            }};

            btn.addEventListener('mousedown', startDrive);
            btn.addEventListener('mouseup', stopDrive);
            btn.addEventListener('mouseleave', stopDrive);
            btn.addEventListener('touchstart', startDrive);
            btn.addEventListener('touchend', stopDrive);
        }});

        // Speed slider
        document.getElementById('speedSlider').addEventListener('input', (e) => {{
            speed = parseInt(e.target.value);
            document.getElementById('speedValue').textContent = speed;
        }});

        // View controls
        function setStream(url) {{
            document.getElementById('stream').src = url + '?t=' + Date.now();
        }}

        function toggleFullscreen() {{
            const img = document.getElementById('stream');
            if (document.fullscreenElement) {{
                document.exitFullscreen();
            }} else {{
                img.requestFullscreen();
            }}
        }}

        // Initialize
        connect();
    </script>
</body>
</html>
"""
        return web.Response(text=html, content_type="text/html")

    async def _generate_mjpeg(self, get_frame_func):
        """Generate MJPEG stream"""
        boundary = "frame"

        while self.running:
            frame = get_frame_func()
            if frame is not None:
                jpeg = self.encode_frame(frame)
                yield (
                    f"--{boundary}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode() + jpeg + b"\r\n"

            await asyncio.sleep(1.0 / self.fps)

    async def _handle_stream(self, request):
        """Stream stereo (side-by-side)"""
        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
            }
        )
        await response.prepare(request)

        async for chunk in self._generate_mjpeg(self.read_stereo_frame):
            await response.write(chunk)

        return response

    async def _handle_stream_left(self, request):
        """Stream left camera only"""
        def get_left():
            if len(self.cameras) > 0:
                ret, frame = self.cameras[0].read()
                return frame if ret else None
            return None

        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache",
            }
        )
        await response.prepare(request)

        async for chunk in self._generate_mjpeg(get_left):
            await response.write(chunk)

        return response

    async def _handle_stream_right(self, request):
        """Stream right camera only"""
        def get_right():
            if len(self.cameras) > 1:
                ret, frame = self.cameras[1].read()
                return frame if ret else None
            return None

        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache",
            }
        )
        await response.prepare(request)

        async for chunk in self._generate_mjpeg(get_right):
            await response.write(chunk)

        return response

    async def _handle_snapshot(self, request):
        """Return single JPEG snapshot"""
        frame = self.read_stereo_frame()
        if frame is not None:
            jpeg = self.encode_frame(frame, quality=95)
            return web.Response(body=jpeg, content_type="image/jpeg")
        return web.Response(status=503, text="Camera not available")

    async def _handle_health(self, request):
        """Health check"""
        return web.json_response({
            "status": "ok",
            "cameras": len(self.cameras),
            "running": self.running
        })

    def run(self):
        """Start server"""
        import numpy as np  # Import here to avoid startup delay
        globals()['np'] = np

        logger.info("Opening cameras...")
        if not self.open_cameras():
            logger.error("No cameras available!")
            return

        self.running = True
        logger.success(f"MJPEG Server starting on http://{self.host}:{self.port}")
        logger.info(f"  Stream:   http://{self.host}:{self.port}/stream")
        logger.info(f"  Snapshot: http://{self.host}:{self.port}/snapshot")

        try:
            web.run_app(self.app, host=self.host, port=self.port,
                       print=lambda x: None)  # Suppress aiohttp startup message
        finally:
            self.running = False
            for cap in self.cameras:
                cap.release()
            logger.info("Server stopped")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="CLOVER MJPEG Streaming Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8080, help="Port")
    parser.add_argument("--width", type=int, default=1280, help="Camera width")
    parser.add_argument("--height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=int, default=30, help="Framerate")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality (1-100)")

    args = parser.parse_args()

    server = MJPEGServer(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality
    )

    server.run()


if __name__ == "__main__":
    main()
