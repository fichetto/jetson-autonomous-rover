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

# Import robust motor controller
try:
    from ..control.modbus_motors import RobustModbusController
except ImportError:
    import sys
    sys.path.insert(0, '/home/jetsonnano/autonomous-rover')
    from src.control.modbus_motors import RobustModbusController


class PIDController:
    """
    Simple PID controller for smooth distance/angle control.
    """
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -255, output_max: float = 255,
                 integral_max: float = 100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, error: float) -> float:
        """
        Update PID controller with new error.

        Args:
            error: Current error (setpoint - measured)

        Returns:
            Control output
        """
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            dt = 0.01  # Minimum dt

        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self.output_min, min(self.output_max, output))

        # Update state
        self.last_error = error
        self.last_time = current_time

        return output


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
    """YOLOv8 cat detector with image preprocessing for low light"""

    CAT_CLASS_ID = 15  # COCO class for cat

    def __init__(self, model_path: str = "models/yolo11n.pt",
                 conf_threshold: float = 0.4,  # Lowered for better detection
                 enhance_image: bool = True,
                 debug_dir: str = None):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.enhance_image = enhance_image
        self.model = None
        self.debug_dir = debug_dir
        self._debug_frame_count = 0

        # CLAHE for contrast enhancement (works great in low light)
        # Higher clipLimit = more contrast, but can cause noise
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        if self.enhance_image:
            logger.info("CLAHE image enhancement ENABLED (clipLimit=3.0)")
        else:
            logger.info("CLAHE image enhancement DISABLED")

        self._load_model()

    def preprocess_image(self, image: np.ndarray, save_debug: bool = False) -> np.ndarray:
        """
        Enhance image for better detection in low light.

        Uses CLAHE (Contrast Limited Adaptive Histogram Equalization)
        on the luminance channel to improve contrast without oversaturating.
        """
        if not self.enhance_image:
            return image

        # Convert to LAB color space
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

        # Split channels
        l, a, b = cv2.split(lab)

        # Apply CLAHE to L channel (luminance)
        l_enhanced = self.clahe.apply(l)

        # Merge back
        lab_enhanced = cv2.merge([l_enhanced, a, b])

        # Convert back to BGR
        enhanced = cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)

        # Save debug comparison periodically
        self._debug_frame_count += 1
        if save_debug or (self.debug_dir and self._debug_frame_count % 100 == 1):
            try:
                from pathlib import Path
                debug_path = Path(self.debug_dir) if self.debug_dir else Path("/tmp")
                debug_path.mkdir(exist_ok=True)
                # Side-by-side comparison
                comparison = np.hstack([image, enhanced])
                cv2.putText(comparison, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(comparison, "CLAHE Enhanced", (image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imwrite(str(debug_path / "clahe_comparison.jpg"), comparison)
                logger.debug(f"Saved CLAHE comparison to {debug_path / 'clahe_comparison.jpg'}")
            except Exception as e:
                logger.warning(f"Could not save debug image: {e}")

        return enhanced

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
        Detect cats in image with preprocessing for low light.

        Args:
            image: BGR image

        Returns:
            List of cat detections with bbox and confidence
        """
        if self.model is None:
            return []

        try:
            # Preprocess image for better detection
            enhanced = self.preprocess_image(image)

            # Run inference on enhanced image
            results = self.model(enhanced, verbose=False, conf=self.conf_threshold)

            cats = []
            all_detections = []  # For debug logging
            # COCO class names for common indoor objects
            COCO_NAMES = {0: 'person', 15: 'cat', 16: 'dog', 17: 'horse',
                          56: 'chair', 57: 'couch', 58: 'potted_plant', 59: 'bed',
                          60: 'dining_table', 62: 'tv', 63: 'laptop', 67: 'cell_phone'}

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = COCO_NAMES.get(cls, f"class_{cls}")
                    all_detections.append((cls, class_name, conf))
                    if cls == self.CAT_CLASS_ID:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cats.append({
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'confidence': conf
                        })
                        logger.info(f"CAT detected! conf={conf:.2f}")

            # Log what was detected (periodically)
            if self._debug_frame_count % 30 == 0 and all_detections:
                det_str = ", ".join([f"{name}({conf:.2f})" for _, name, conf in all_detections[:5]])
                logger.debug(f"Detections: {det_str}")

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


class CatFollowerState:
    """State machine states"""
    PATROL = "PATROL"      # Searching for cat (rotating)
    FOLLOW = "FOLLOW"      # Following detected cat
    IDLE = "IDLE"          # Waiting


class CatFollower:
    """
    Autonomous cat following controller with patrol behavior

    State Machine:
    - PATROL: Rotate 360° searching for cats
    - FOLLOW: Track and follow detected cat, keeping it centered
    - Returns to PATROL after losing cat for some time
    """

    # Patrol parameters (counter-rotation mode)
    PATROL_ROTATION_SPEED = 40      # Speed percentage (0-100%)
    PATROL_SEGMENT_DURATION = 3.0   # Seconds per rotation segment before switching direction
    PATROL_INTERVAL = 60.0          # Seconds between patrol cycles
    CAT_LOST_TIMEOUT = 2.0          # Seconds before returning to patrol when cat lost

    def __init__(self,
                 target_distance: float = 0.6,  # 60cm (small environment)
                 distance_tolerance: float = 0.1,  # 10cm
                 angle_tolerance: float = 5.0,  # degrees (tighter for centering)
                 max_speed: int = 200,  # Increased for faster response
                 camera_fov: float = 62.0):  # IMX219 horizontal FOV

        self.target_distance = target_distance
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        self.max_speed = max_speed
        self.camera_fov = camera_fov

        # Components
        self.detector: Optional[CatDetector] = None
        self.depth_estimator: Optional[StereoDepthEstimator] = None
        self.motors: Optional[RobustModbusController] = None

        # State machine
        self.state = CatFollowerState.PATROL
        self.state_start_time = 0.0
        self.last_cat_seen_time = 0.0
        self.last_patrol_time = 0.0
        self.patrol_direction = 1  # 1 = right, -1 = left

        # Running state
        self.running = False
        self.current_cat: Optional[CatDetection] = None
        self.frame_width = 640  # Updated for 640x480
        self.frame_height = 480

        # Control gains
        self.kp_distance = 180  # Forward/back gain (fast approach)
        self.kp_angle = 4.0     # Rotation gain (smoother steering)

        # Obstacle avoidance parameters
        self.obstacle_threshold = 0.4  # Stop if obstacle closer than 40cm
        self.obstacle_slowdown = 0.6   # Slow down if obstacle closer than 60cm
        self.obstacle_detected = False
        self.last_depth_map = None

        # Last command values (percentages, -100 to +100)
        self.last_forward = 0
        self.last_rotation = 0

        # PID Controllers for smooth control
        # Distance PID: aggressive for fast approach
        self.distance_pid = PIDController(
            kp=200.0,    # Higher P for faster response
            ki=15.0,     # Integral gain (reduces steady-state error)
            kd=40.0,     # Derivative gain (reduces overshoot)
            output_min=-max_speed,
            output_max=max_speed,
            integral_max=60  # Limit integral windup
        )

        # Angle PID: balanced response (not too jerky)
        self.angle_pid = PIDController(
            kp=4.0,      # Reduced for smoother steering
            ki=0.3,      # Small I
            kd=1.5,      # D for damping oscillations
            output_min=-max_speed,
            output_max=max_speed,
            integral_max=30
        )

        logger.info(f"CatFollower initialized: target={target_distance}m, "
                   f"tolerance={distance_tolerance}m, obstacle_threshold={self.obstacle_threshold}m")
        logger.info(f"PID controllers: distance(P={self.distance_pid.kp}), angle(P={self.angle_pid.kp})")
        logger.info(f"Patrol mode: counter-rotation every {self.PATROL_INTERVAL}s, "
                   f"speed={self.PATROL_ROTATION_SPEED}%, segment={self.PATROL_SEGMENT_DURATION}s")

    def initialize(self, serial_port: str = '/dev/ttyUSB0') -> bool:
        """Initialize all components"""
        try:
            # Initialize cat detector with CLAHE enhancement
            logger.info("Initializing cat detector with CLAHE enhancement...")
            self.detector = CatDetector(
                model_path="/home/jetsonnano/autonomous-rover/models/yolo11n.pt",
                conf_threshold=0.35,  # Lowered for better detection in poor conditions
                enhance_image=True,
                debug_dir="/home/jetsonnano/autonomous-rover/debug_images"
            )

            # Initialize depth estimator with REAL calibration
            logger.info("Initializing depth estimator...")
            calibration = StereoCalibration.load_from_file()
            self.depth_estimator = StereoDepthEstimator(calibration)

            # Initialize robust motor controller (thread-based)
            logger.info("Initializing motor controller (robust thread-based)...")
            self.motors = RobustModbusController(serial_port)
            if not self.motors.connect():
                logger.error("Failed to connect to motors")
                return False

            return True

        except Exception as e:
            logger.error(f"Initialization error: {e}")
            return False

    def check_obstacles(self, depth_map: np.ndarray) -> bool:
        """
        Check for obstacles in the path

        Checks the lower center region of the depth map where obstacles
        would be in the rover's path.

        Args:
            depth_map: Depth map in meters

        Returns:
            True if obstacle detected within threshold
        """
        h, w = depth_map.shape[:2]

        # Check lower third, center portion (where rover is heading)
        y1 = int(h * 0.5)  # Lower half
        y2 = h
        x1 = int(w * 0.25)  # Center 50%
        x2 = int(w * 0.75)

        roi = depth_map[y1:y2, x1:x2]
        valid = roi[(roi > 0.1) & (roi < 10.0)]  # Valid depth readings

        if len(valid) < 100:  # Not enough valid pixels
            return False

        # Use 10th percentile (closest obstacles)
        min_distance = np.percentile(valid, 10)

        return min_distance < self.obstacle_threshold

    def get_obstacle_speed_factor(self, depth_map: np.ndarray) -> float:
        """
        Get speed reduction factor based on obstacle proximity

        Returns:
            Factor 0.0-1.0 (0=stop, 1=full speed)
        """
        h, w = depth_map.shape[:2]

        # Check center lower region
        y1 = int(h * 0.5)
        y2 = h
        x1 = int(w * 0.25)
        x2 = int(w * 0.75)

        roi = depth_map[y1:y2, x1:x2]
        valid = roi[(roi > 0.1) & (roi < 10.0)]

        if len(valid) < 100:
            return 1.0  # No valid data, assume clear

        min_distance = np.percentile(valid, 10)

        if min_distance < self.obstacle_threshold:
            return 0.0  # Stop!
        elif min_distance < self.obstacle_slowdown:
            # Linear slowdown between threshold and slowdown distance
            factor = (min_distance - self.obstacle_threshold) / (self.obstacle_slowdown - self.obstacle_threshold)
            return max(0.3, factor)  # Minimum 30% speed

        return 1.0  # Full speed

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
        self.last_depth_map = depth_map

        # Check for obstacles
        self.obstacle_detected = self.check_obstacles(depth_map)

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

        # Apply obstacle avoidance
        if self.last_depth_map is not None:
            speed_factor = self.get_obstacle_speed_factor(self.last_depth_map)

            # Only reduce forward speed, not backward
            if forward_speed > 0:
                forward_speed = int(forward_speed * speed_factor)

                if speed_factor < 0.5:
                    logger.warning(f"Obstacle detected! Reducing speed to {speed_factor*100:.0f}%")

        return forward_speed, rotation_speed

    # ==================== STATE MACHINE ====================

    def change_state(self, new_state: str):
        """Transition to a new state"""
        if new_state != self.state:
            logger.info(f"State: {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = time.time()
            # Reset PID controllers on state change
            self.distance_pid.reset()
            self.angle_pid.reset()

    def patrol_behavior(self):
        """
        Patrol behavior: counter-rotation search for cats.
        Alternates rotation direction every segment.
        Uses high-level motor commands.
        """
        elapsed = time.time() - self.state_start_time
        segment_elapsed = elapsed % (self.PATROL_SEGMENT_DURATION * 2)

        # Determine current rotation direction based on time
        if segment_elapsed < self.PATROL_SEGMENT_DURATION:
            # First half: rotate right
            self.motors.rotate_right(self.PATROL_ROTATION_SPEED)
            direction = "RIGHT"
        else:
            # Second half: rotate left
            self.motors.rotate_left(self.PATROL_ROTATION_SPEED)
            direction = "LEFT"

        # Check if patrol cycle is complete (4 segments = 2 full counter-rotations)
        if elapsed > self.PATROL_SEGMENT_DURATION * 4:
            self.last_patrol_time = time.time()
            self.motors.stop()
            self.change_state(CatFollowerState.IDLE)
            return

        logger.debug(f"[PATROL] Rotating {direction} at {self.PATROL_ROTATION_SPEED}%")

    def follow_behavior(self, cat: CatDetection):
        """
        Follow behavior using PID controllers and high-level commands.
        PRIORITY: Centering first, then distance!

        Args:
            cat: Detected cat
        """
        # Update last seen time
        self.last_cat_seen_time = time.time()

        # ROTATION: Use PID for smooth centering
        # Error is angle offset (positive = cat to the right)
        rotation_raw = self.angle_pid.update(cat.angle_offset)

        # If cat is way off center, boost rotation
        is_way_off = abs(cat.angle_offset) > 15.0

        if is_way_off:
            rotation_raw = rotation_raw * 1.4

        # Convert to percentage (-100 to +100)
        rotation_percent = int(max(-100, min(100, rotation_raw * 100 / self.max_speed)))

        # FORWARD/BACKWARD: Use PID for smooth distance control
        distance_error = cat.distance - self.target_distance

        if is_way_off:
            # Cat way off center - focus on rotation, stop forward
            forward_percent = 0
            self.distance_pid.integral *= 0.5
        elif abs(distance_error) < self.distance_tolerance:
            # Within tolerance - stop
            forward_percent = 0
            self.distance_pid.integral = 0
        else:
            # Use PID for distance control
            forward_raw = self.distance_pid.update(distance_error)

            # Reduce forward when also rotating (prioritize centering)
            rotation_factor = 1.0 - min(abs(rotation_percent) / 100, 0.6)
            forward_raw = forward_raw * rotation_factor

            # Convert to percentage
            forward_percent = int(max(-100, min(100, forward_raw * 100 / self.max_speed)))

        # Apply obstacle avoidance
        if self.last_depth_map is not None and forward_percent > 0:
            speed_factor = self.get_obstacle_speed_factor(self.last_depth_map)
            forward_percent = int(forward_percent * speed_factor)

        # Send high-level command
        self.motors.drive(forward_percent, rotation_percent)

        # Store for logging
        self.last_forward = forward_percent
        self.last_rotation = rotation_percent

    def update_state_machine(self, cat: Optional[CatDetection]):
        """
        Main state machine update.
        Calls motor commands directly using high-level API.

        Args:
            cat: Detected cat (or None)
        """
        current_time = time.time()

        # ===== CAT DETECTED: STOP PATROL AND FOLLOW =====
        if cat is not None:
            # Cat detected! Immediately switch to FOLLOW
            if self.state != CatFollowerState.FOLLOW:
                self.motors.stop()  # Stop any patrol rotation first
                self.change_state(CatFollowerState.FOLLOW)
            self.follow_behavior(cat)
            return

        # ===== NO CAT DETECTED =====
        time_since_cat = current_time - self.last_cat_seen_time

        if self.state == CatFollowerState.FOLLOW:
            # Was following, cat lost
            if time_since_cat > self.CAT_LOST_TIMEOUT:
                logger.info(f"Cat lost for {time_since_cat:.1f}s, starting patrol")
                self.motors.stop()
                self.change_state(CatFollowerState.PATROL)
            else:
                # Keep reduced command briefly (cat might reappear)
                self.motors.drive(
                    int(self.last_forward * 0.5),
                    int(self.last_rotation * 0.8)
                )

        elif self.state == CatFollowerState.IDLE:
            # Check if it's time for patrol
            time_since_patrol = current_time - self.last_patrol_time
            if time_since_patrol > self.PATROL_INTERVAL:
                logger.info(f"Starting patrol (last: {time_since_patrol:.0f}s ago)")
                self.change_state(CatFollowerState.PATROL)
            else:
                self.motors.stop()

        elif self.state == CatFollowerState.PATROL:
            self.patrol_behavior()

    # ==================== END STATE MACHINE ====================

    def apply_control(self, forward: int, rotation: int):
        """
        Apply motor control with smoothing.

        The RobustModbusController handles continuous command refresh
        in a background thread at 20Hz, so we just set the target speeds.

        Args:
            forward: Forward speed (-255 to +255)
            rotation: Rotation speed (-255 to +255, positive = right)
        """
        if self.motors is None:
            return

        # Apply smoothing (exponential moving average)
        forward = int(self.smoothing_factor * self.last_forward + (1 - self.smoothing_factor) * forward)
        rotation = int(self.smoothing_factor * self.last_rotation + (1 - self.smoothing_factor) * rotation)
        self.last_forward = forward
        self.last_rotation = rotation

        # Use the controller's differential drive method
        self.motors.set_differential(forward, rotation)

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

        # Draw obstacle warning
        if self.obstacle_detected:
            cv2.putText(result, "OBSTACLE!", (self.frame_width - 150, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # Draw obstacle zone
            h, w = self.frame_height, self.frame_width
            y1 = int(h * 0.5)
            x1, x2 = int(w * 0.25), int(w * 0.75)
            cv2.rectangle(result, (x1, y1), (x2, h), (0, 0, 255), 2)

        return result

    def draw_overlay_with_state(self, image: np.ndarray, cat: Optional[CatDetection]) -> np.ndarray:
        """Draw debug overlay with state machine info"""
        result = self.draw_overlay(image, cat)

        # Draw state indicator
        state_colors = {
            CatFollowerState.PATROL: (0, 255, 255),   # Yellow
            CatFollowerState.FOLLOW: (0, 255, 0),     # Green
            CatFollowerState.IDLE: (128, 128, 128),   # Gray
        }
        color = state_colors.get(self.state, (255, 255, 255))

        # State box
        cv2.rectangle(result, (self.frame_width - 120, 5), (self.frame_width - 5, 35), color, -1)
        cv2.putText(result, self.state, (self.frame_width - 115, 28),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Patrol indicator
        if self.state == CatFollowerState.PATROL:
            elapsed = time.time() - self.state_start_time
            progress = min(elapsed / self.PATROL_DURATION, 1.0)
            bar_width = int(100 * progress)
            cv2.rectangle(result, (10, self.frame_height - 40),
                         (10 + bar_width, self.frame_height - 30), (0, 255, 255), -1)
            cv2.rectangle(result, (10, self.frame_height - 40),
                         (110, self.frame_height - 30), (255, 255, 255), 1)
            direction = "→" if self.patrol_direction > 0 else "←"
            cv2.putText(result, f"Patrol {direction}", (120, self.frame_height - 32),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return result

    def run_with_cameras(self, left_id: int = 1, right_id: int = 0,
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
            fps=30,
            flip_method=2  # 180° rotation for upside-down cameras
        ) as camera:
            self.running = True
            self._frame_count = 0

            # Initialize state machine
            self.state_start_time = time.time()
            self.last_cat_seen_time = 0
            self.last_patrol_time = time.time() - self.PATROL_INTERVAL  # Start with patrol

            logger.info("State machine started in PATROL mode")
            self.change_state(CatFollowerState.PATROL)

            try:
                while self.running:
                    ret, frame = camera.read()
                    if not ret:
                        continue

                    # Process frame and detect cat
                    cat = self.process_frame(frame.left, frame.right)

                    # Update state machine (motor commands are called internally)
                    self.update_state_machine(cat)

                    # Log status periodically
                    self._frame_count += 1
                    if self._frame_count % 10 == 0:
                        if cat is not None:
                            logger.info(f"[{self.state}] Cat at {cat.distance:.2f}m, "
                                       f"{cat.angle_offset:.1f}° -> fwd={self.last_forward}%, rot={self.last_rotation}%")

                    # Show preview
                    if show_preview:
                        preview = self.draw_overlay_with_state(frame.left, cat)
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
            self.motors.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Cat Follower for CLOVER Rover")
    parser.add_argument('--target-distance', type=float, default=0.6,
                       help='Target following distance in meters (default: 0.6)')
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
        angle_tolerance=8.0,   # Slightly relaxed for faster movement
        max_speed=200  # Fast response
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
