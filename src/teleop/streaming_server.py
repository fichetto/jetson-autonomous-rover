#!/usr/bin/env python3
"""
WebRTC Stereo Video Streaming Server for Meta Quest 3
Low-latency H.264 streaming with hardware encoding on Jetson Orin Nano
"""

import asyncio
import json
import cv2
import numpy as np
import fractions
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass
from loguru import logger

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame

import sys
sys.path.insert(0, '/home/jetsonnano/autonomous-rover')
from src.vision.stereo_camera import StereoCamera, StereoFrame, FrameConsumer, FramePriority


@dataclass
class StreamConfig:
    """Streaming configuration"""
    host: str = "0.0.0.0"
    port: int = 8080
    video_width: int = 2560      # Side-by-side: 1280 * 2
    video_height: int = 720
    video_fps: int = 30
    video_bitrate: int = 8000000  # 8 Mbps for stereo
    stereo_mode: str = "sbs"      # "sbs" (side-by-side) or "ou" (over-under)


class StereoVideoTrack(VideoStreamTrack):
    """
    WebRTC video track that streams stereo frames from StereoCamera
    """

    kind = "video"

    def __init__(self, stereo_camera: StereoCamera, config: StreamConfig):
        super().__init__()
        self.stereo_camera = stereo_camera
        self.config = config
        self._frame_count = 0
        self._start_time = time.time()

        # Register as consumer
        stereo_camera.register_callback("webrtc_stream", self._on_frame)
        self._latest_frame: Optional[StereoFrame] = None
        self._frame_lock = asyncio.Lock()

        logger.info(f"StereoVideoTrack initialized: {config.video_width}x{config.video_height}@{config.video_fps}fps")

    def _on_frame(self, frame: StereoFrame):
        """Callback for new stereo frames"""
        self._latest_frame = frame

    async def recv(self) -> VideoFrame:
        """
        Receive next video frame for WebRTC
        Called by aiortc at the configured framerate
        """
        pts, time_base = await self.next_timestamp()

        # Get latest stereo frame
        stereo_frame = self._latest_frame

        if stereo_frame is None:
            # No frame yet, return black frame
            black = np.zeros((self.config.video_height, self.config.video_width, 3), dtype=np.uint8)
            frame = VideoFrame.from_ndarray(black, format="bgr24")
        else:
            # Create stereo image based on mode
            if self.config.stereo_mode == "sbs":
                combined = self.stereo_camera.create_side_by_side(stereo_frame)
            else:
                combined = self.stereo_camera.create_over_under(stereo_frame)

            frame = VideoFrame.from_ndarray(combined, format="bgr24")

        frame.pts = pts
        frame.time_base = time_base
        self._frame_count += 1

        return frame

    def stop(self):
        """Stop the video track"""
        self.stereo_camera.unregister_callback("webrtc_stream")
        super().stop()


class TeleopStreamingServer:
    """
    WebRTC streaming server for CLOVER teleoperation
    Provides stereo video stream and control WebSocket
    """

    def __init__(self, stereo_camera: StereoCamera, config: Optional[StreamConfig] = None):
        """
        Initialize streaming server

        Args:
            stereo_camera: StereoCamera instance for frame source
            config: StreamConfig or None for defaults
        """
        self.stereo_camera = stereo_camera
        self.config = config or StreamConfig()
        self.app = web.Application()
        self.pcs: Dict[str, RTCPeerConnection] = {}
        self.relay = MediaRelay()

        self._setup_routes()
        logger.info(f"TeleopStreamingServer initialized on {self.config.host}:{self.config.port}")

    def _setup_routes(self):
        """Setup HTTP routes"""
        self.app.router.add_get("/", self._handle_index)
        self.app.router.add_get("/health", self._handle_health)
        self.app.router.add_post("/offer", self._handle_offer)
        self.app.router.add_get("/stats", self._handle_stats)
        self.app.router.add_static("/static", "/home/jetsonnano/autonomous-rover/src/teleop/static")

        # CORS middleware
        self.app.middlewares.append(self._cors_middleware)

    @web.middleware
    async def _cors_middleware(self, request, handler):
        """Add CORS headers for Quest browser access"""
        response = await handler(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return response

    async def _handle_index(self, request: web.Request) -> web.Response:
        """Serve index page with connection info"""
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>CLOVER Teleop</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
        </head>
        <body style="background:#1a1a2e;color:#eee;font-family:system-ui;padding:20px;">
            <h1>🤖 CLOVER Teleoperation Server</h1>
            <p>WebRTC Stereo Streaming Active</p>
            <ul>
                <li><strong>Video:</strong> {self.config.video_width}x{self.config.video_height}@{self.config.video_fps}fps</li>
                <li><strong>Stereo Mode:</strong> {self.config.stereo_mode.upper()}</li>
                <li><strong>Bitrate:</strong> {self.config.video_bitrate // 1000000} Mbps</li>
            </ul>
            <h3>Endpoints:</h3>
            <ul>
                <li><code>POST /offer</code> - WebRTC SDP offer</li>
                <li><code>GET /stats</code> - Server statistics</li>
                <li><code>GET /health</code> - Health check</li>
            </ul>
            <p>Connect with Meta Quest 3 Unity app for VR control.</p>
        </body>
        </html>
        """
        return web.Response(text=html, content_type="text/html")

    async def _handle_health(self, request: web.Request) -> web.Response:
        """Health check endpoint"""
        return web.json_response({
            "status": "ok",
            "camera_running": self.stereo_camera._running,
            "active_connections": len(self.pcs)
        })

    async def _handle_stats(self, request: web.Request) -> web.Response:
        """Return server statistics"""
        return web.json_response({
            "camera": self.stereo_camera.get_properties(),
            "consumers": self.stereo_camera.get_consumers_info(),
            "connections": len(self.pcs),
            "config": {
                "video_width": self.config.video_width,
                "video_height": self.config.video_height,
                "video_fps": self.config.video_fps,
                "stereo_mode": self.config.stereo_mode
            }
        })

    async def _handle_offer(self, request: web.Request) -> web.Response:
        """
        Handle WebRTC offer from client
        Returns SDP answer for peer connection
        """
        try:
            params = await request.json()
            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

            # Create peer connection
            pc = RTCPeerConnection()
            pc_id = f"pc_{len(self.pcs)}_{int(time.time())}"
            self.pcs[pc_id] = pc

            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"Connection {pc_id} state: {pc.connectionState}")
                if pc.connectionState == "failed" or pc.connectionState == "closed":
                    await self._cleanup_peer(pc_id)

            @pc.on("track")
            def on_track(track):
                logger.info(f"Received track: {track.kind}")

            # Add video track
            video_track = StereoVideoTrack(self.stereo_camera, self.config)
            pc.addTrack(video_track)

            # Handle offer and create answer
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

            logger.info(f"WebRTC connection established: {pc_id}")

            return web.json_response({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })

        except Exception as e:
            logger.error(f"Error handling offer: {e}")
            return web.json_response({"error": str(e)}, status=500)

    async def _cleanup_peer(self, pc_id: str):
        """Cleanup peer connection"""
        if pc_id in self.pcs:
            pc = self.pcs.pop(pc_id)
            await pc.close()
            logger.info(f"Cleaned up connection: {pc_id}")

    async def start(self):
        """Start the streaming server"""
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.config.host, self.config.port)
        await site.start()
        logger.success(f"Streaming server running on http://{self.config.host}:{self.config.port}")

    def run(self):
        """Run server (blocking)"""
        web.run_app(self.app, host=self.config.host, port=self.config.port)

    async def shutdown(self):
        """Shutdown server and cleanup connections"""
        for pc_id in list(self.pcs.keys()):
            await self._cleanup_peer(pc_id)
        logger.info("Streaming server shutdown complete")


# Standalone test
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CLOVER WebRTC Streaming Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8080, help="Port")
    parser.add_argument("--width", type=int, default=1280, help="Camera width")
    parser.add_argument("--height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=int, default=30, help="Framerate")
    args = parser.parse_args()

    config = StreamConfig(
        host=args.host,
        port=args.port,
        video_width=args.width * 2,  # Side-by-side
        video_height=args.height,
        video_fps=args.fps
    )

    # Create stereo camera
    stereo = StereoCamera(
        left_sensor_id=0,
        right_sensor_id=1,
        width=args.width,
        height=args.height,
        fps=args.fps
    )

    # Start camera capture
    stereo.start()

    # Create and run server
    server = TeleopStreamingServer(stereo, config)

    try:
        server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        stereo.release()
