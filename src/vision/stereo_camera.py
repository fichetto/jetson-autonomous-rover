#!/usr/bin/env python3
"""
Stereo camera capture with frame distribution for Jetson Orin Nano
Supports multiple consumers: VR streaming, object detection, autonomous navigation
"""

import cv2
import numpy as np
import threading
import time
from typing import Optional, Tuple, Callable, Dict, List
from dataclasses import dataclass
from enum import Enum, auto
from loguru import logger
from abc import ABC, abstractmethod


@dataclass
class StereoFrame:
    """Container for synchronized stereo frames"""
    left: np.ndarray
    right: np.ndarray
    timestamp: float
    frame_id: int


class FramePriority(Enum):
    """Priority levels for frame consumers"""
    CRITICAL = auto()   # VR streaming - lowest latency
    HIGH = auto()       # Autonomous navigation
    NORMAL = auto()     # Object detection
    LOW = auto()        # Recording, logging


class FrameConsumer(ABC):
    """Abstract base class for frame consumers"""

    def __init__(self, name: str, priority: FramePriority = FramePriority.NORMAL):
        self.name = name
        self.priority = priority
        self.frames_received = 0
        self.frames_dropped = 0
        self._enabled = True

    @abstractmethod
    def process_frame(self, frame: StereoFrame) -> None:
        """Process a stereo frame - implement in subclass"""
        pass

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    @property
    def is_enabled(self) -> bool:
        return self._enabled


class StereoCamera:
    """
    Synchronized dual IMX219 camera capture with frame distribution
    Uses GStreamer with NVIDIA hardware acceleration
    """

    def __init__(
        self,
        left_sensor_id: int = 0,
        right_sensor_id: int = 1,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        flip_method: int = 0
    ):
        """
        Initialize stereo camera pair

        Args:
            left_sensor_id: CSI sensor ID for left camera
            right_sensor_id: CSI sensor ID for right camera
            width: Frame width
            height: Frame height
            fps: Target framerate
            flip_method: Image flip method (0=none, 2=180 degrees)
        """
        self.left_sensor_id = left_sensor_id
        self.right_sensor_id = right_sensor_id
        self.width = width
        self.height = height
        self.fps = fps
        self.flip_method = flip_method

        self.cap_left: Optional[cv2.VideoCapture] = None
        self.cap_right: Optional[cv2.VideoCapture] = None

        # Frame distribution
        self._consumers: Dict[str, FrameConsumer] = {}
        self._callbacks: Dict[str, Callable[[StereoFrame], None]] = {}
        self._consumer_lock = threading.Lock()

        # Async capture
        self._running = False
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[StereoFrame] = None
        self._frame_count = 0
        self._capture_thread: Optional[threading.Thread] = None

        # Stats
        self._fps_actual = 0.0
        self._last_fps_time = time.time()
        self._fps_frame_count = 0

        logger.info(f"StereoCamera initialized: L={left_sensor_id}, R={right_sensor_id}, "
                   f"{width}x{height}@{fps}fps")

    def _gstreamer_pipeline(self, sensor_id: int) -> str:
        """
        Generate GStreamer pipeline for CSI camera with hardware acceleration
        """
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, "
            f"framerate={self.fps}/1, format=NV12 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=1 max-buffers=2"
        )

    def open(self) -> bool:
        """Open both camera streams"""
        try:
            # Open left camera
            pipeline_left = self._gstreamer_pipeline(self.left_sensor_id)
            logger.debug(f"Left camera pipeline: {pipeline_left}")
            self.cap_left = cv2.VideoCapture(pipeline_left, cv2.CAP_GSTREAMER)

            if not self.cap_left.isOpened():
                logger.error("Failed to open left camera")
                return False

            # Open right camera
            pipeline_right = self._gstreamer_pipeline(self.right_sensor_id)
            logger.debug(f"Right camera pipeline: {pipeline_right}")
            self.cap_right = cv2.VideoCapture(pipeline_right, cv2.CAP_GSTREAMER)

            if not self.cap_right.isOpened():
                logger.error("Failed to open right camera")
                self.cap_left.release()
                return False

            logger.success("Stereo cameras opened successfully")
            return True

        except Exception as e:
            logger.error(f"Stereo camera initialization error: {e}")
            return False

    def read(self) -> Tuple[bool, Optional[StereoFrame]]:
        """Read synchronized frames from both cameras"""
        if self.cap_left is None or self.cap_right is None:
            return False, None

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            return False, None

        # Read from both cameras as close together as possible
        ret_left, frame_left = self.cap_left.read()
        ret_right, frame_right = self.cap_right.read()
        timestamp = time.time()

        if not ret_left or not ret_right:
            logger.warning("Failed to read stereo frame")
            return False, None

        self._frame_count += 1
        self._update_fps_stats()

        return True, StereoFrame(
            left=frame_left,
            right=frame_right,
            timestamp=timestamp,
            frame_id=self._frame_count
        )

    def _update_fps_stats(self):
        """Update FPS statistics"""
        self._fps_frame_count += 1
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed >= 1.0:
            self._fps_actual = self._fps_frame_count / elapsed
            self._fps_frame_count = 0
            self._last_fps_time = now

    # ==================== Frame Distribution ====================

    def register_consumer(self, consumer: FrameConsumer) -> bool:
        """
        Register a frame consumer

        Args:
            consumer: FrameConsumer instance

        Returns:
            True if registered successfully
        """
        with self._consumer_lock:
            if consumer.name in self._consumers:
                logger.warning(f"Consumer '{consumer.name}' already registered")
                return False

            self._consumers[consumer.name] = consumer
            logger.info(f"Registered consumer: {consumer.name} (priority: {consumer.priority.name})")
            return True

    def unregister_consumer(self, name: str) -> bool:
        """Unregister a frame consumer by name"""
        with self._consumer_lock:
            if name in self._consumers:
                del self._consumers[name]
                logger.info(f"Unregistered consumer: {name}")
                return True
            return False

    def register_callback(self, name: str, callback: Callable[[StereoFrame], None]) -> bool:
        """
        Register a simple callback function for frame delivery

        Args:
            name: Unique identifier for the callback
            callback: Function that takes a StereoFrame

        Returns:
            True if registered successfully
        """
        with self._consumer_lock:
            if name in self._callbacks:
                logger.warning(f"Callback '{name}' already registered")
                return False

            self._callbacks[name] = callback
            logger.info(f"Registered callback: {name}")
            return True

    def unregister_callback(self, name: str) -> bool:
        """Unregister a callback by name"""
        with self._consumer_lock:
            if name in self._callbacks:
                del self._callbacks[name]
                logger.info(f"Unregistered callback: {name}")
                return True
            return False

    def _distribute_frame(self, frame: StereoFrame):
        """
        Distribute frame to all registered consumers and callbacks
        Consumers are processed in priority order
        """
        with self._consumer_lock:
            # Sort consumers by priority
            sorted_consumers = sorted(
                self._consumers.values(),
                key=lambda c: c.priority.value
            )

            # Distribute to consumers
            for consumer in sorted_consumers:
                if consumer.is_enabled:
                    try:
                        consumer.process_frame(frame)
                        consumer.frames_received += 1
                    except Exception as e:
                        consumer.frames_dropped += 1
                        logger.error(f"Consumer '{consumer.name}' error: {e}")

            # Call callbacks
            for name, callback in self._callbacks.items():
                try:
                    callback(frame)
                except Exception as e:
                    logger.error(f"Callback '{name}' error: {e}")

    def get_consumers_info(self) -> List[dict]:
        """Get info about registered consumers"""
        with self._consumer_lock:
            return [
                {
                    'name': c.name,
                    'priority': c.priority.name,
                    'enabled': c.is_enabled,
                    'frames_received': c.frames_received,
                    'frames_dropped': c.frames_dropped
                }
                for c in self._consumers.values()
            ]

    # ==================== Async Capture ====================

    def start(self):
        """Start asynchronous frame capture and distribution"""
        if self._running:
            logger.warning("Capture already running")
            return

        if not self.open():
            logger.error("Failed to open cameras")
            return

        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        logger.info("Stereo capture started")

    def _capture_loop(self):
        """Background capture and distribution loop"""
        while self._running:
            ret, frame = self.read()
            if ret and frame is not None:
                # Update latest frame for polling
                with self._frame_lock:
                    self._latest_frame = frame

                # Distribute to all consumers
                self._distribute_frame(frame)

    def stop(self):
        """Stop capture"""
        self._running = False
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)
        logger.info("Stereo capture stopped")

    def get_latest_frame(self) -> Optional[StereoFrame]:
        """Get the most recent captured frame (non-blocking)"""
        with self._frame_lock:
            return self._latest_frame

    def release(self):
        """Release camera resources"""
        self.stop()

        if self.cap_left is not None:
            self.cap_left.release()
        if self.cap_right is not None:
            self.cap_right.release()

        logger.info("Stereo cameras released")

    # ==================== Utility Methods ====================

    def create_side_by_side(self, frame: StereoFrame) -> np.ndarray:
        """Create side-by-side stereo image for VR display"""
        return np.hstack((frame.left, frame.right))

    def create_over_under(self, frame: StereoFrame) -> np.ndarray:
        """Create over-under stereo image"""
        return np.vstack((frame.left, frame.right))

    def get_properties(self) -> dict:
        """Get camera properties"""
        return {
            'left_sensor_id': self.left_sensor_id,
            'right_sensor_id': self.right_sensor_id,
            'width': self.width,
            'height': self.height,
            'fps_target': self.fps,
            'fps_actual': round(self._fps_actual, 1),
            'stereo_width': self.width * 2,
            'frame_count': self._frame_count,
            'is_running': self._running,
            'consumers_count': len(self._consumers),
            'callbacks_count': len(self._callbacks)
        }

    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.release()


# ==================== Example Consumers ====================

class VRStreamConsumer(FrameConsumer):
    """Consumer for VR streaming - minimal latency"""

    def __init__(self, on_frame: Callable[[StereoFrame], None]):
        super().__init__("vr_stream", FramePriority.CRITICAL)
        self.on_frame = on_frame

    def process_frame(self, frame: StereoFrame) -> None:
        self.on_frame(frame)


class DetectionConsumer(FrameConsumer):
    """Consumer for object detection"""

    def __init__(self, detector=None):
        super().__init__("detection", FramePriority.NORMAL)
        self.detector = detector
        self.last_detections = []

    def process_frame(self, frame: StereoFrame) -> None:
        if self.detector is not None:
            # Use left camera for detection
            self.last_detections = self.detector.detect(frame.left)


class NavigationConsumer(FrameConsumer):
    """Consumer for autonomous navigation"""

    def __init__(self, navigation_callback: Callable[[StereoFrame], None]):
        super().__init__("navigation", FramePriority.HIGH)
        self.navigation_callback = navigation_callback

    def process_frame(self, frame: StereoFrame) -> None:
        self.navigation_callback(frame)


if __name__ == "__main__":
    # Test stereo camera with multiple consumers
    logger.info("Testing stereo camera with frame distribution...")

    # Example callback functions
    vr_frames = []
    nav_frames = []

    def vr_callback(frame: StereoFrame):
        vr_frames.append(frame.frame_id)

    def nav_callback(frame: StereoFrame):
        nav_frames.append(frame.frame_id)

    with StereoCamera(
        left_sensor_id=0,
        right_sensor_id=1,
        width=1280,
        height=720,
        fps=30
    ) as stereo:

        # Register consumers
        vr_consumer = VRStreamConsumer(vr_callback)
        nav_consumer = NavigationConsumer(nav_callback)

        stereo.register_consumer(vr_consumer)
        stereo.register_consumer(nav_consumer)

        # Also register a simple callback
        stereo.register_callback("logger", lambda f: None)

        logger.info(f"Camera properties: {stereo.get_properties()}")
        logger.info(f"Consumers: {stereo.get_consumers_info()}")

        # Let it run for a few seconds
        time.sleep(3)

        logger.info(f"VR frames received: {len(vr_frames)}")
        logger.info(f"Nav frames received: {len(nav_frames)}")
        logger.info(f"Final stats: {stereo.get_properties()}")

        logger.success("Stereo camera distribution test completed")
