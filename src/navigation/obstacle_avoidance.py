#!/usr/bin/env python3
"""
Obstacle Avoidance with Sensor Fusion
Combines ultrasonic sensors and vision for safe navigation
"""

import numpy as np
from typing import Dict, Tuple, Optional, List
from dataclasses import dataclass
from loguru import logger

from ..sensors.ultrasonic import SensorReading
from ..vision.detector import Detection


@dataclass
class NavigationCommand:
    """Navigation command output"""
    vx: float  # Forward velocity (m/s)
    vy: float  # Lateral velocity (m/s)
    wz: float  # Angular velocity (rad/s)
    action: str  # Action description


class ObstacleAvoidance:
    """
    Dynamic obstacle avoidance controller
    Fuses ultrasonic and vision data for safe navigation
    """

    def __init__(self, config: Dict):
        """
        Initialize obstacle avoidance

        Args:
            config: Navigation configuration
        """
        self.config = config
        self.safety_distance = config.get('obstacle_avoidance', {}).get('safety_distance', 0.5)
        self.critical_distance = config.get('obstacle_avoidance', {}).get('critical_distance', 0.3)
        self.avoidance_speed_factor = config.get('obstacle_avoidance', {}).get('avoidance_speed_factor', 0.6)

        self.max_linear_speed = config.get('mechanum', {}).get('max_speed', 1.0)
        self.max_angular_speed = 1.0  # rad/s

        logger.info(f"Obstacle avoidance initialized "
                   f"(safety: {self.safety_distance}m, critical: {self.critical_distance}m)")

    def compute_avoidance(self,
                         sensor_readings: Dict[str, SensorReading],
                         vision_detections: List[Detection],
                         target_velocity: Tuple[float, float, float]
                         ) -> NavigationCommand:
        """
        Compute safe navigation command with obstacle avoidance

        Args:
            sensor_readings: Ultrasonic sensor data
            vision_detections: Visual object detections
            target_velocity: Desired velocity (vx, vy, wz)

        Returns:
            Safe navigation command
        """
        target_vx, target_vy, target_wz = target_velocity

        # Check for critical obstacles
        critical_obstacle = self._check_critical_obstacles(sensor_readings)

        if critical_obstacle:
            logger.warning(f"Critical obstacle detected: {critical_obstacle}")
            return NavigationCommand(0.0, 0.0, 0.0, "EMERGENCY_STOP")

        # Compute obstacle vectors from sensors
        obstacle_vector = self._compute_obstacle_vector(sensor_readings)

        # Compute avoidance velocity
        avoid_vx, avoid_vy = self._compute_avoidance_velocity(obstacle_vector)

        # Blend target velocity with avoidance
        final_vx = target_vx * 0.7 + avoid_vx * 0.3
        final_vy = target_vy * 0.7 + avoid_vy * 0.3
        final_wz = target_wz

        # Check if path is blocked
        if self._is_path_blocked(sensor_readings, (final_vx, final_vy)):
            # Try alternative paths
            alternative = self._find_alternative_path(sensor_readings)
            if alternative:
                final_vx, final_vy, final_wz = alternative
                action = "AVOIDING"
            else:
                # No clear path, stop
                final_vx, final_vy, final_wz = 0.0, 0.0, 0.0
                action = "BLOCKED"
        else:
            action = "NAVIGATING"

        # Apply speed limits
        speed = np.sqrt(final_vx**2 + final_vy**2)
        if speed > self.max_linear_speed:
            scale = self.max_linear_speed / speed
            final_vx *= scale
            final_vy *= scale

        final_wz = np.clip(final_wz, -self.max_angular_speed, self.max_angular_speed)

        return NavigationCommand(final_vx, final_vy, final_wz, action)

    def _check_critical_obstacles(self, readings: Dict[str, SensorReading]) -> Optional[str]:
        """
        Check for critical obstacles requiring immediate stop

        Args:
            readings: Sensor readings

        Returns:
            Name of critical sensor or None
        """
        for name, reading in readings.items():
            if reading.valid and reading.distance < self.critical_distance:
                return name
        return None

    def _compute_obstacle_vector(self, readings: Dict[str, SensorReading]
                                ) -> Tuple[float, float]:
        """
        Compute repulsive vector from obstacles using potential field

        Args:
            readings: Sensor readings

        Returns:
            Repulsive vector (x, y)
        """
        repulsion_x = 0.0
        repulsion_y = 0.0

        for name, reading in readings.items():
            if not reading.valid or reading.distance > self.safety_distance:
                continue

            # Repulsive force inversely proportional to distance
            force_magnitude = (self.safety_distance - reading.distance) / self.safety_distance

            # Direction away from obstacle
            angle_rad = np.radians(reading.position.angle)
            obs_x = reading.position.x + reading.distance * np.cos(angle_rad)
            obs_y = reading.position.y + reading.distance * np.sin(angle_rad)

            # Vector from obstacle to robot center
            dx = -obs_x
            dy = -obs_y
            dist = np.sqrt(dx**2 + dy**2)

            if dist > 0:
                repulsion_x += (dx / dist) * force_magnitude
                repulsion_y += (dy / dist) * force_magnitude

        return repulsion_x, repulsion_y

    def _compute_avoidance_velocity(self, obstacle_vector: Tuple[float, float]
                                   ) -> Tuple[float, float]:
        """
        Convert obstacle vector to avoidance velocity

        Args:
            obstacle_vector: Repulsive vector from obstacles

        Returns:
            Avoidance velocity (vx, vy)
        """
        rep_x, rep_y = obstacle_vector

        # Scale to avoidance speed
        magnitude = np.sqrt(rep_x**2 + rep_y**2)

        if magnitude > 0:
            avoid_vx = (rep_x / magnitude) * self.max_linear_speed * self.avoidance_speed_factor
            avoid_vy = (rep_y / magnitude) * self.max_linear_speed * self.avoidance_speed_factor
        else:
            avoid_vx, avoid_vy = 0.0, 0.0

        return avoid_vx, avoid_vy

    def _is_path_blocked(self, readings: Dict[str, SensorReading],
                        velocity: Tuple[float, float]) -> bool:
        """
        Check if intended path is blocked

        Args:
            readings: Sensor readings
            velocity: Intended velocity (vx, vy)

        Returns:
            True if path is blocked
        """
        vx, vy = velocity

        if abs(vx) < 0.01 and abs(vy) < 0.01:
            return False

        # Determine primary movement direction
        angle = np.arctan2(vy, vx)
        angle_deg = np.degrees(angle)

        # Check sensors in movement direction
        for name, reading in readings.items():
            if not reading.valid:
                continue

            sensor_angle = reading.position.angle
            angle_diff = abs(self._normalize_angle(angle_deg - sensor_angle))

            # If sensor is in movement direction and detects obstacle
            if angle_diff < 45 and reading.distance < self.safety_distance:
                return True

        return False

    def _find_alternative_path(self, readings: Dict[str, SensorReading]
                              ) -> Optional[Tuple[float, float, float]]:
        """
        Find alternative path when main path is blocked

        Args:
            readings: Sensor readings

        Returns:
            Alternative velocity (vx, vy, wz) or None
        """
        # Evaluate clearance in different directions
        directions = [
            (0, 'front'),
            (90, 'left'),
            (-90, 'right'),
            (45, 'front_left'),
            (-45, 'front_right'),
        ]

        best_direction = None
        best_clearance = 0.0

        for angle_deg, sector in directions:
            clearance = self._get_clearance_in_direction(readings, angle_deg)

            if clearance > best_clearance and clearance > self.safety_distance:
                best_clearance = clearance
                best_direction = angle_deg

        if best_direction is not None:
            # Generate velocity in best direction
            angle_rad = np.radians(best_direction)
            speed = self.max_linear_speed * self.avoidance_speed_factor

            vx = speed * np.cos(angle_rad)
            vy = speed * np.sin(angle_rad)
            wz = 0.0  # No rotation while avoiding

            logger.info(f"Alternative path found: {best_direction}° (clearance: {best_clearance:.2f}m)")
            return vx, vy, wz

        return None

    def _get_clearance_in_direction(self, readings: Dict[str, SensorReading],
                                    direction_deg: float) -> float:
        """
        Get clearance distance in specific direction

        Args:
            readings: Sensor readings
            direction_deg: Direction angle in degrees

        Returns:
            Minimum clearance distance
        """
        min_clearance = float('inf')

        for name, reading in readings.items():
            if not reading.valid:
                continue

            sensor_angle = reading.position.angle
            angle_diff = abs(self._normalize_angle(direction_deg - sensor_angle))

            # Consider sensors within 45° of target direction
            if angle_diff < 45:
                weight = 1.0 - (angle_diff / 45.0)  # Closer sensors have more weight
                weighted_distance = reading.distance * weight
                min_clearance = min(min_clearance, weighted_distance)

        return min_clearance if min_clearance != float('inf') else 0.0

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


if __name__ == "__main__":
    # Test obstacle avoidance
    config = {
        'obstacle_avoidance': {
            'safety_distance': 0.5,
            'critical_distance': 0.3,
            'avoidance_speed_factor': 0.6
        },
        'mechanum': {
            'max_speed': 1.0
        }
    }

    avoidance = ObstacleAvoidance(config)
    logger.info("Obstacle avoidance module loaded")
