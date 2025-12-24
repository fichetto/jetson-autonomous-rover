#!/usr/bin/env python3
"""
Ultrasonic Sensor Manager
Handles 6 ultrasonic sensors with sensor fusion
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from loguru import logger


@dataclass
class SensorPosition:
    """Sensor position and orientation"""
    x: float  # meters, relative to robot center
    y: float  # meters, relative to robot center
    angle: float  # degrees, 0=forward


@dataclass
class SensorReading:
    """Single sensor reading"""
    distance: float  # meters
    position: SensorPosition
    timestamp: float
    valid: bool


class UltrasonicSensorArray:
    """
    Manages array of 6 ultrasonic sensors
    """

    def __init__(self, config: Dict):
        """
        Initialize sensor array

        Args:
            config: Sensor configuration dict
        """
        self.config = config
        self.max_range = config.get('max_range', 4.0)
        self.min_range = config.get('min_range', 0.02)
        self.fov = config.get('fov', 15)  # field of view in degrees

        # Initialize sensor positions
        self.sensors: Dict[str, SensorPosition] = {}
        positions = config.get('positions', {})

        for name, pos in positions.items():
            self.sensors[name] = SensorPosition(
                x=pos[0],
                y=pos[1],
                angle=pos[2]
            )

        logger.info(f"Initialized {len(self.sensors)} ultrasonic sensors")

    def process_readings(self, raw_data: Dict[str, float],
                        timestamp: float) -> Dict[str, SensorReading]:
        """
        Process raw sensor data

        Args:
            raw_data: Dictionary of sensor name -> distance (meters)
            timestamp: Reading timestamp

        Returns:
            Dictionary of processed sensor readings
        """
        readings = {}

        for name, distance in raw_data.items():
            if name not in self.sensors:
                logger.warning(f"Unknown sensor: {name}")
                continue

            # Validate reading
            valid = self.min_range <= distance <= self.max_range

            readings[name] = SensorReading(
                distance=distance,
                position=self.sensors[name],
                timestamp=timestamp,
                valid=valid
            )

        return readings

    def get_closest_obstacle(self, readings: Dict[str, SensorReading]
                            ) -> Optional[Tuple[str, float]]:
        """
        Find closest detected obstacle

        Args:
            readings: Sensor readings

        Returns:
            Tuple of (sensor_name, distance) or None
        """
        valid_readings = {
            name: reading.distance
            for name, reading in readings.items()
            if reading.valid
        }

        if not valid_readings:
            return None

        closest = min(valid_readings.items(), key=lambda x: x[1])
        return closest

    def get_sector_distances(self, readings: Dict[str, SensorReading]
                            ) -> Dict[str, float]:
        """
        Get minimum distance for each sector (front, rear, left, right)

        Args:
            readings: Sensor readings

        Returns:
            Dictionary with sector distances
        """
        sectors = {
            'front': float('inf'),
            'rear': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }

        for name, reading in readings.items():
            if not reading.valid:
                continue

            angle = reading.position.angle
            distance = reading.distance

            # Classify by angle
            if -45 <= angle <= 45:
                sectors['front'] = min(sectors['front'], distance)
            elif 135 <= angle or angle <= -135:
                sectors['rear'] = min(sectors['rear'], distance)
            elif 45 < angle < 135:
                sectors['left'] = min(sectors['left'], distance)
            else:
                sectors['right'] = min(sectors['right'], distance)

        return sectors

    def create_occupancy_grid(self, readings: Dict[str, SensorReading],
                             grid_size: Tuple[int, int] = (50, 50),
                             resolution: float = 0.1) -> np.ndarray:
        """
        Create local occupancy grid from sensor readings

        Args:
            readings: Sensor readings
            grid_size: Grid dimensions (rows, cols)
            resolution: Grid cell size in meters

        Returns:
            2D numpy array with occupancy probabilities
        """
        grid = np.zeros(grid_size, dtype=np.float32)
        center = (grid_size[0] // 2, grid_size[1] // 2)

        for name, reading in readings.items():
            if not reading.valid:
                continue

            # Convert sensor reading to grid coordinates
            angle_rad = np.radians(reading.position.angle)

            # Obstacle position
            obs_x = reading.position.x + reading.distance * np.cos(angle_rad)
            obs_y = reading.position.y + reading.distance * np.sin(angle_rad)

            # Convert to grid indices
            grid_x = int(center[0] + obs_x / resolution)
            grid_y = int(center[1] + obs_y / resolution)

            # Mark obstacle (with some uncertainty)
            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                grid[grid_x, grid_y] = 1.0

                # Spread occupancy due to sensor FOV
                fov_cells = int(np.tan(np.radians(self.fov / 2)) * reading.distance / resolution)
                for dx in range(-fov_cells, fov_cells + 1):
                    for dy in range(-fov_cells, fov_cells + 1):
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < grid_size[0] and 0 <= ny < grid_size[1]:
                            prob = 0.5 * np.exp(-(dx**2 + dy**2) / (2 * fov_cells**2))
                            grid[nx, ny] = max(grid[nx, ny], prob)

        return grid

    def check_safe_direction(self, readings: Dict[str, SensorReading],
                            direction: str, safety_distance: float = 0.5) -> bool:
        """
        Check if movement in direction is safe

        Args:
            readings: Sensor readings
            direction: 'forward', 'backward', 'left', 'right'
            safety_distance: Minimum safe distance in meters

        Returns:
            True if safe to move
        """
        sectors = self.get_sector_distances(readings)

        direction_map = {
            'forward': 'front',
            'backward': 'rear',
            'left': 'left',
            'right': 'right'
        }

        sector = direction_map.get(direction.lower())
        if sector is None:
            logger.warning(f"Unknown direction: {direction}")
            return False

        min_distance = sectors.get(sector, float('inf'))
        return min_distance > safety_distance

    def estimate_free_space(self, readings: Dict[str, SensorReading]
                           ) -> Dict[str, float]:
        """
        Estimate free space around robot

        Args:
            readings: Sensor readings

        Returns:
            Dictionary with free space estimates per direction
        """
        free_space = {}

        for name, reading in readings.items():
            if reading.valid:
                free_space[name] = reading.distance
            else:
                free_space[name] = self.max_range

        return free_space


if __name__ == "__main__":
    # Test sensor processing
    config = {
        'max_range': 4.0,
        'min_range': 0.02,
        'fov': 15,
        'positions': {
            'front_left': [0.15, 0.12, 30],
            'front_center': [0.18, 0.0, 0],
            'front_right': [0.15, -0.12, -30],
            'rear_left': [-0.15, 0.12, 150],
            'rear_center': [-0.18, 0.0, 180],
            'rear_right': [-0.15, -0.12, -150],
        }
    }

    sensor_array = UltrasonicSensorArray(config)

    # Simulate readings
    raw_data = {
        'front_left': 1.5,
        'front_center': 2.0,
        'front_right': 1.8,
        'rear_left': 3.0,
        'rear_center': 2.5,
        'rear_right': 3.2,
    }

    import time
    readings = sensor_array.process_readings(raw_data, time.time())

    logger.info(f"Closest obstacle: {sensor_array.get_closest_obstacle(readings)}")
    logger.info(f"Sector distances: {sensor_array.get_sector_distances(readings)}")
    logger.info(f"Safe forward: {sensor_array.check_safe_direction(readings, 'forward')}")
