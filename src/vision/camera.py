#!/usr/bin/env python3
"""
Camera capture with GStreamer hardware acceleration
Optimized for Jetson Orin Nano
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from loguru import logger


class JetsonCamera:
    """
    Camera interface with GStreamer hardware acceleration
    """

    def __init__(self, device_id: int = 0, width: int = 1280, height: int = 720,
                 fps: int = 30, use_gstreamer: bool = True):
        """
        Initialize camera

        Args:
            device_id: Camera device ID
            width: Frame width
            height: Frame height
            fps: Frames per second
            use_gstreamer: Use GStreamer hardware acceleration
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.use_gstreamer = use_gstreamer
        self.cap: Optional[cv2.VideoCapture] = None

        logger.info(f"Initializing camera {device_id} ({width}x{height}@{fps}fps)")

    def _gstreamer_pipeline(self) -> str:
        """
        Generate GStreamer pipeline for hardware-accelerated capture

        Returns:
            GStreamer pipeline string
        """
        # Try MIPI CSI camera first, fallback to USB
        # This pipeline uses NVIDIA hardware acceleration
        pipeline = (
            f"nvarguscamerasrc sensor-id={self.device_id} ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, "
            f"framerate={self.fps}/1, format=NV12 ! "
            f"nvvidconv ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=1"
        )
        return pipeline

    def _usb_gstreamer_pipeline(self) -> str:
        """
        Generate GStreamer pipeline for USB camera

        Returns:
            GStreamer pipeline string
        """
        pipeline = (
            f"v4l2src device=/dev/video{self.device_id} ! "
            f"video/x-raw, width={self.width}, height={self.height}, "
            f"framerate={self.fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=1"
        )
        return pipeline

    def open(self) -> bool:
        """
        Open camera stream

        Returns:
            True if successful
        """
        try:
            if self.use_gstreamer:
                # Try CSI camera first
                pipeline = self._gstreamer_pipeline()
                logger.debug(f"Trying GStreamer CSI pipeline: {pipeline}")
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

                if not self.cap.isOpened():
                    # Fallback to USB camera
                    logger.info("CSI camera not available, trying USB camera")
                    pipeline = self._usb_gstreamer_pipeline()
                    logger.debug(f"Trying GStreamer USB pipeline: {pipeline}")
                    self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

                if not self.cap.isOpened():
                    logger.warning("GStreamer failed, falling back to V4L2")
                    self.use_gstreamer = False

            if not self.use_gstreamer or not self.cap.isOpened():
                # Fallback to standard V4L2
                self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            if self.cap.isOpened():
                logger.success(f"Camera opened successfully "
                             f"({'GStreamer' if self.use_gstreamer else 'V4L2'})")
                return True
            else:
                logger.error("Failed to open camera")
                return False

        except Exception as e:
            logger.error(f"Camera initialization error: {e}")
            return False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Read frame from camera

        Returns:
            Tuple of (success, frame)
        """
        if self.cap is None or not self.cap.isOpened():
            return False, None

        ret, frame = self.cap.read()

        if not ret:
            logger.warning("Failed to read frame")
            return False, None

        return True, frame

    def release(self):
        """Release camera resources"""
        if self.cap is not None:
            self.cap.release()
            logger.info("Camera released")

    def get_properties(self) -> dict:
        """
        Get current camera properties

        Returns:
            Dictionary with camera properties
        """
        if self.cap is None or not self.cap.isOpened():
            return {}

        return {
            'width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'fps': int(self.cap.get(cv2.CAP_PROP_FPS)),
            'backend': 'GStreamer' if self.use_gstreamer else 'V4L2'
        }

    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.release()


if __name__ == "__main__":
    # Test camera capture
    logger.info("Testing camera capture...")

    with JetsonCamera(device_id=0, width=1280, height=720, fps=30) as camera:
        if camera.cap.isOpened():
            logger.info(f"Camera properties: {camera.get_properties()}")

            # Capture a few test frames
            for i in range(10):
                ret, frame = camera.read()
                if ret:
                    logger.info(f"Frame {i}: shape={frame.shape}, dtype={frame.dtype}")
                else:
                    logger.error(f"Failed to capture frame {i}")
                    break

            logger.success("Camera test completed")
        else:
            logger.error("Failed to open camera")
