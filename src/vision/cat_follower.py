#!/usr/bin/env python3
"""
Cat Follower for CLOVER Rover
=============================
Detects cats and follows them at a specified distance using:
- YOLOv11n for cat detection (COCO class 15)
- Stereo depth estimation for distance measurement
- PID-like control for smooth following

Usage:
    python3 cat_follower.py --target-distance 0.5
"""

import cv2
import numpy as np
import time
import argparse
import subprocess
import serial
import threading
from typing import Optional, Tuple, List
from dataclasses import dataclass
from loguru import logger

# Try to import ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logger.warning("Ultralytics not available, using ONNX detector")

try:
    from .stereo_depth import StereoDepthEstimator, StereoCalibration
except ImportError:
    from stereo_depth import StereoDepthEstimator, StereoCalibration


@dataclass
class CatDetection:
    """Detected cat with position and distance"""
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    confidence: float
    center_x: int
    center_y: int
    distance: float  # meters
    angle_offset: float  # degrees from center


class CatDetector:
    """YOLOv8 cat detector"""

    CAT_CLASS_ID = 15  # COCO class for cat

    def __init__(self, model_path: str = "models/yolo11n.pt",
                 conf_threshold: float = 0.5):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None

        self._load_model()

    def _load_model(self):
        """Load YOLO model"""
        if YOLO_AVAILABLE:
            try:
                # Use .pt for better performance if available
                pt_path = self.model_path.replace('.onnx', '.pt')
                self.model = YOLO(pt_path)
                logger.success(f"Loaded YOLO model: {pt_path}")
            except Exception as e:
                logger.error(f"Failed to load model: {e}")
        else:
            logger.warning("Using fallback detector (implement ONNX inference)")

    def detect_cats(self, image: np.ndarray) -> List[dict]:
        """
        Detect cats in image

        Args:
            image: BGR image

        Returns:
            List of cat detections with bbox and confidence
        """
        if self.model is None:
            return []

        try:
            # Run inference
            results = self.model(image, verbose=False, conf=self.conf_threshold)

            cats = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    if cls == self.CAT_CLASS_ID:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cats.append({
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'confidence': conf
                        })

            return cats

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []


class ModbusMotorController:
    """Simple Modbus motor controller"""

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False

    def connect(self) -> bool:
        """Connect to Arduino"""
        try:
            subprocess.run(['stty', '-F', self.port, '-hupcl'],
                         capture_output=True, timeout=5)
            self.serial = serial.Serial()
            self.serial.port = self.port
            self.serial.baudrate = self.baudrate
            self.serial.timeout = 0.3
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.open()
            time.sleep(2)
            self.connected = True
            logger.success(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Connection error: {e}")
            return False

    def close(self):
        """Close connection"""
        if self.serial:
            self.stop()
            self.serial.close()
            self.connected = False

    def _crc16(self, data):
        """Calculate Modbus CRC16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def set_motors(self, fl: int, fr: int, rl: int, rr: int):
        """Set motor speeds (-255 to +255)"""
        if not self.connected:
            return False

        try:
            values = [fl + 255, fr + 255, rl + 255, rr + 255]
            request = bytes([1, 0x10, 0, 0, 0, 4, 8])
            for val in values:
                request += bytes([(val >> 8) & 0xFF, val & 0xFF])
            crc = self._crc16(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
            self.serial.reset_input_buffer()
            self.serial.write(request)
            self.serial.flush()
            time.sleep(0.03)
            self.serial.read(8)
            return True
        except:
            return False

    def stop(self):
        """Stop all motors"""
        return self.set_motors(0, 0, 0, 0)

    def move_forward(self, speed: int):
        """Move forward"""
        return self.set_motors(speed, speed, speed, speed)

    def move_backward(self, speed: int):
        """Move backward"""
        return self.set_motors(-speed, -speed, -speed, -speed)

    def rotate_left(self, speed: int):
        """Rotate left (counter-clockwise)"""
        return self.set_motors(-speed, speed, -speed, speed)

    def rotate_right(self, speed: int):
        """Rotate right (clockwise)"""
        return self.set_motors(speed, -speed, speed, -speed)


class CatFollower:
    """
    Main cat following controller

    Uses a simple control strategy:
    1. Detect cats in frame
    2. Get distance to closest cat
    3. Adjust forward/backward to maintain target distance
    4. Adjust rotation to keep cat centered
    """

    def __init__(self,
                 target_distance: float = 0.5,  # 50cm
                 distance_tolerance: float = 0.1,  # 10cm
                 angle_tolerance: float = 10.0,  # degrees
                 max_speed: int = 150,  # ~60% PWM (minimum to move rover)
                 camera_fov: float = 62.0):  # IMX219 horizontal FOV

        self.target_distance = target_distance
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.max_speed = max_speed
        self.camera_fov = camera_fov

        # Components
        self.detector: Optional[CatDetector] = None
        self.depth_estimator: Optional[StereoDepthEstimator] = None
        self.motors: Optional[ModbusMotorController] = None

        # State
        self.running = False
        self.current_cat: Optional[CatDetection] = None
        self.frame_width = 1280
        self.frame_height = 720

        # Control gains
        self.kp_distance = 150  # Forward/back gain
        self.kp_angle = 2.0     # Rotation gain

        logger.info(f"CatFollower initialized: target={target_distance}m, "
                   f"tolerance={distance_tolerance}m")

    def initialize(self, serial_port: str = '/dev/ttyUSB0') -> bool:
        """Initialize all components"""
        try:
            # Initialize cat detector
            logger.info("Initializing cat detector...")
            self.detector = CatDetector(
                model_path="/home/jetsonnano/autonomous-rover/models/yolo11n.pt"
            )

            # Initialize depth estimator
            logger.info("Initializing depth estimator...")
            calibration = StereoCalibration(
                baseline=0.06,  # 6cm - adjust for your camera setup!
                focal_length=500.0,  # Calibrate for your camera
            )
            self.depth_estimator = StereoDepthEstimator(calibration)

            # Initialize motor controller
            logger.info("Initializing motor controller...")
            self.motors = ModbusMotorController(serial_port)
            if not self.motors.connect():
                logger.error("Failed to connect to motors")
                return False

            return True

        except Exception as e:
            logger.error(f"Initialization error: {e}")
            return False

    def process_frame(self, left: np.ndarray, right: np.ndarray) -> Optional[CatDetection]:
        """
        Process stereo frame and detect/track cat

        Args:
            left: Left camera image
            right: Right camera image

        Returns:
            CatDetection if cat found, None otherwise
        """
        self.frame_height, self.frame_width = left.shape[:2]

        # Detect cats in left image
        cats = self.detector.detect_cats(left)

        if not cats:
            self.current_cat = None
            return None

        # Get depth map
        depth_map = self.depth_estimator.compute_depth(left, right)

        # Find closest cat
        best_cat = None
        best_distance = float('inf')

        for cat in cats:
            bbox = cat['bbox']
            distance = self.depth_estimator.get_depth_in_bbox(depth_map, bbox)

            if distance < best_distance:
                best_distance = distance
                x1, y1, x2, y2 = bbox
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Calculate angle offset from center
                pixel_offset = center_x - (self.frame_width // 2)
                angle_offset = (pixel_offset / self.frame_width) * self.camera_fov

                best_cat = CatDetection(
                    bbox=bbox,
                    confidence=cat['confidence'],
                    center_x=center_x,
                    center_y=center_y,
                    distance=distance,
                    angle_offset=angle_offset
                )

        self.current_cat = best_cat
        return best_cat

    def compute_control(self, cat: CatDetection) -> Tuple[int, int]:
        """
        Compute motor control based on cat position

        Args:
            cat: Detected cat

        Returns:
            (forward_speed, rotation_speed)
        """
        # Distance control (forward/backward)
        distance_error = cat.distance - self.target_distance

        if abs(distance_error) < self.distance_tolerance:
            forward_speed = 0
        else:
            forward_speed = int(np.clip(
                self.kp_distance * distance_error,
                -self.max_speed,
                self.max_speed
            ))

        # Angle control (rotation)
        if abs(cat.angle_offset) < self.angle_tolerance:
            rotation_speed = 0
        else:
            rotation_speed = int(np.clip(
                self.kp_angle * cat.angle_offset,
                -self.max_speed,
                self.max_speed
            ))

        return forward_speed, rotation_speed

    def apply_control(self, forward: int, rotation: int):
        """
        Apply motor control

        Args:
            forward: Forward speed (-255 to +255)
            rotation: Rotation speed (-255 to +255, positive = right)
        """
        if self.motors is None:
            return

        # Differential drive mixing
        left_speed = forward - rotation
        right_speed = forward + rotation

        # Clamp speeds
        left_speed = int(np.clip(left_speed, -255, 255))
        right_speed = int(np.clip(right_speed, -255, 255))

        self.motors.set_motors(left_speed, right_speed, left_speed, right_speed)

    def draw_overlay(self, image: np.ndarray, cat: Optional[CatDetection]) -> np.ndarray:
        """Draw debug overlay on image"""
        result = image.copy()

        # Draw target zone
        center_x = self.frame_width // 2
        tolerance_pixels = int((self.angle_tolerance / self.camera_fov) * self.frame_width)
        cv2.line(result, (center_x - tolerance_pixels, 0),
                (center_x - tolerance_pixels, self.frame_height), (0, 255, 255), 1)
        cv2.line(result, (center_x + tolerance_pixels, 0),
                (center_x + tolerance_pixels, self.frame_height), (0, 255, 255), 1)

        if cat is not None:
            x1, y1, x2, y2 = cat.bbox

            # Draw bounding box
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(result, (cat.center_x, cat.center_y), 5, (0, 0, 255), -1)

            # Draw info
            info = f"Cat: {cat.distance:.2f}m, {cat.angle_offset:.1f}deg"
            cv2.putText(result, info, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Draw distance indicator
            if cat.distance < self.target_distance - self.distance_tolerance:
                status = "TOO CLOSE - BACK"
                color = (0, 0, 255)
            elif cat.distance > self.target_distance + self.distance_tolerance:
                status = "TOO FAR - FORWARD"
                color = (255, 0, 0)
            else:
                status = "GOOD DISTANCE"
                color = (0, 255, 0)

            cv2.putText(result, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        else:
            cv2.putText(result, "NO CAT DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Draw target distance
        cv2.putText(result, f"Target: {self.target_distance}m",
                   (10, self.frame_height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return result

    def run_with_cameras(self, left_id: int = 0, right_id: int = 1,
                        show_preview: bool = True):
        """
        Run cat follower with stereo cameras

        Args:
            left_id: Left camera sensor ID
            right_id: Right camera sensor ID
            show_preview: Show preview window
        """
        try:
            from .stereo_camera import StereoCamera
        except ImportError:
            from stereo_camera import StereoCamera

        logger.info("Starting cat follower with stereo cameras...")

        with StereoCamera(
            left_sensor_id=left_id,
            right_sensor_id=right_id,
            width=640,  # Lower resolution for faster processing
            height=480,
            fps=30
        ) as camera:
            self.running = True
            no_cat_frames = 0

            try:
                while self.running:
                    ret, frame = camera.read()
                    if not ret:
                        continue

                    # Process frame
                    cat = self.process_frame(frame.left, frame.right)

                    if cat is not None:
                        no_cat_frames = 0
                        forward, rotation = self.compute_control(cat)
                        self.apply_control(forward, rotation)
                        logger.debug(f"Cat at {cat.distance:.2f}m, {cat.angle_offset:.1f}deg "
                                   f"-> fwd={forward}, rot={rotation}")
                    else:
                        no_cat_frames += 1
                        # Stop if cat lost for too long
                        if no_cat_frames > 10:
                            self.motors.stop()

                    # Show preview
                    if show_preview:
                        preview = self.draw_overlay(frame.left, cat)
                        cv2.imshow("Cat Follower", preview)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

            except KeyboardInterrupt:
                logger.info("Interrupted by user")

            finally:
                self.running = False
                self.motors.stop()
                if show_preview:
                    cv2.destroyAllWindows()

    def run_with_video(self, video_path: str, show_preview: bool = True):
        """
        Test with video file (no motors, simulation only)

        Args:
            video_path: Path to video file
            show_preview: Show preview window
        """
        logger.info(f"Running simulation with video: {video_path}")

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            logger.error("Cannot open video file")
            return

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop
                    continue

                # Use same frame for both (no stereo in video)
                cat = self.process_frame(frame, frame)

                if cat is not None:
                    forward, rotation = self.compute_control(cat)
                    logger.info(f"Cat: {cat.distance:.2f}m -> fwd={forward}, rot={rotation}")

                if show_preview:
                    preview = self.draw_overlay(frame, cat)
                    cv2.imshow("Cat Follower (Simulation)", preview)
                    if cv2.waitKey(30) & 0xFF == ord('q'):
                        break

        finally:
            cap.release()
            cv2.destroyAllWindows()

    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        if self.motors:
            self.motors.stop()
            self.motors.close()


def main():
    parser = argparse.ArgumentParser(description="Cat Follower for CLOVER Rover")
    parser.add_argument('--target-distance', type=float, default=0.5,
                       help='Target following distance in meters (default: 0.5)')
    parser.add_argument('--serial', type=str, default='/dev/ttyUSB0',
                       help='Serial port for Arduino')
    parser.add_argument('--video', type=str, default=None,
                       help='Test with video file instead of cameras')
    parser.add_argument('--no-motors', action='store_true',
                       help='Disable motors (simulation only)')
    parser.add_argument('--no-preview', action='store_true',
                       help='Disable preview window')

    args = parser.parse_args()

    logger.info("=" * 50)
    logger.info("  CLOVER Cat Follower")
    logger.info("=" * 50)
    logger.info(f"Target distance: {args.target_distance}m")

    follower = CatFollower(
        target_distance=args.target_distance,
        distance_tolerance=0.1,
        angle_tolerance=10.0,
        max_speed=80
    )

    try:
        if not args.no_motors:
            if not follower.initialize(args.serial):
                logger.error("Initialization failed")
                return

        if args.video:
            follower.run_with_video(args.video, show_preview=not args.no_preview)
        else:
            follower.run_with_cameras(show_preview=not args.no_preview)

    finally:
        follower.cleanup()
        logger.info("Cat follower stopped")


if __name__ == "__main__":
    main()
