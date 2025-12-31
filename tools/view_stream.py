#!/usr/bin/env python3
"""
Simple WebRTC video viewer for CLOVER stereo stream
Run on any PC to view the rover's camera feed
"""

import asyncio
import argparse
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRecorder
import aiohttp

class VideoViewer:
    def __init__(self, server_url: str):
        self.server_url = server_url
        self.pc = None
        self.frame = None
        self.running = False

    async def connect(self):
        """Connect to CLOVER streaming server"""
        print(f"Connecting to {self.server_url}...")

        self.pc = RTCPeerConnection()

        @self.pc.on("track")
        def on_track(track):
            print(f"Received track: {track.kind}")
            if track.kind == "video":
                asyncio.create_task(self._receive_frames(track))

        @self.pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state: {self.pc.connectionState}")
            if self.pc.connectionState == "failed":
                await self.disconnect()

        # Add transceiver for receiving video
        self.pc.addTransceiver("video", direction="recvonly")

        # Create offer
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)

        # Send to server
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{self.server_url}/offer",
                json={"type": offer.type, "sdp": offer.sdp}
            ) as response:
                if response.status != 200:
                    raise Exception(f"Server error: {response.status}")

                answer = await response.json()

                if "error" in answer:
                    raise Exception(answer["error"])

                await self.pc.setRemoteDescription(
                    RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
                )

        print("Connected!")
        self.running = True

    async def _receive_frames(self, track):
        """Receive and process video frames"""
        while self.running:
            try:
                frame = await track.recv()
                # Convert to numpy array
                img = frame.to_ndarray(format="bgr24")
                self.frame = img
            except Exception as e:
                print(f"Frame error: {e}")
                break

    async def disconnect(self):
        """Disconnect from server"""
        self.running = False
        if self.pc:
            await self.pc.close()
            self.pc = None
        print("Disconnected")

    def get_frame(self):
        """Get latest frame"""
        return self.frame


async def main(args):
    viewer = VideoViewer(args.server)

    try:
        await viewer.connect()

        # Create window
        window_name = "CLOVER Stereo Stream"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        if args.fullscreen:
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        view_mode = "stereo"  # stereo, left, right

        print("\nControls:")
        print("  Q - Quit")
        print("  S - Stereo view")
        print("  L - Left eye only")
        print("  R - Right eye only")
        print("  F - Toggle fullscreen")
        print("  Space - Save screenshot")

        while viewer.running:
            frame = viewer.get_frame()

            if frame is not None:
                display = frame.copy()

                # Apply view mode
                h, w = display.shape[:2]
                if view_mode == "left":
                    display = display[:, :w//2]
                elif view_mode == "right":
                    display = display[:, w//2:]

                # Add info overlay
                if args.info:
                    cv2.putText(display, f"Mode: {view_mode}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(display, f"Size: {w}x{h}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                view_mode = "stereo"
                print("View: Stereo")
            elif key == ord('l'):
                view_mode = "left"
                print("View: Left eye")
            elif key == ord('r'):
                view_mode = "right"
                print("View: Right eye")
            elif key == ord('f'):
                # Toggle fullscreen
                prop = cv2.getWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN)
                if prop == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                else:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            elif key == ord(' '):
                # Screenshot
                if frame is not None:
                    filename = f"clover_screenshot_{int(asyncio.get_event_loop().time())}.png"
                    cv2.imwrite(filename, frame)
                    print(f"Screenshot saved: {filename}")

            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        await viewer.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CLOVER Stereo Stream Viewer")
    parser.add_argument("--server", default="http://localhost:8080",
                       help="Server URL (default: http://localhost:8080)")
    parser.add_argument("--fullscreen", "-f", action="store_true",
                       help="Start in fullscreen mode")
    parser.add_argument("--info", "-i", action="store_true",
                       help="Show info overlay")
    args = parser.parse_args()

    asyncio.run(main(args))
