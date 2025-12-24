#!/usr/bin/env python3
"""
Mechanum Wheel Drive Controller
Handles omnidirectional movement kinematics
"""

import numpy as np
from typing import Tuple, Dict
from dataclasses import dataclass
from loguru import logger


@dataclass
class MechanumConfig:
    """Mechanum drive configuration"""
    wheel_base: float  # meters (front-rear distance)
    track_width: float  # meters (left-right distance)
    wheel_radius: float  # meters
    max_speed: float  # m/s
    max_acceleration: float  # m/s²

    # Motor inversion flags
    invert_front_left: bool = False
    invert_front_right: bool = True
    invert_rear_left: bool = False
    invert_rear_right: bool = True


class MechanumDrive:
    """
    Mechanum wheel drive controller with inverse kinematics

    Mechanum wheels allow omnidirectional movement:
    - Forward/Backward
    - Strafe left/right
    - Rotate in place
    - Any combination of the above
    """

    def __init__(self, config: MechanumConfig):
        """
        Initialize Mechanum drive controller

        Args:
            config: Drive configuration
        """
        self.config = config

        # Kinematic parameters
        self.l = (config.wheel_base + config.track_width) / 2  # Half of (wheelbase + track)
        self.r = config.wheel_radius

        # Current velocity state
        self.current_vx = 0.0  # m/s (forward)
        self.current_vy = 0.0  # m/s (left)
        self.current_wz = 0.0  # rad/s (rotation)

        logger.info("Mechanum drive initialized")
        logger.debug(f"Wheelbase: {config.wheel_base}m, Track: {config.track_width}m")

    def inverse_kinematics(self, vx: float, vy: float, wz: float
                          ) -> Tuple[float, float, float, float]:
        """
        Calculate wheel velocities from desired robot velocity

        Args:
            vx: Desired forward velocity (m/s)
            vy: Desired lateral velocity (m/s, positive = left)
            wz: Desired angular velocity (rad/s, positive = counterclockwise)

        Returns:
            Tuple of wheel velocities (fl, fr, rl, rr) in m/s
        """
        # Mechanum inverse kinematics matrix
        # [vfl]   [1  -1  -l]   [vx]
        # [vfr] = [1   1   l] * [vy]
        # [vrl]   [1   1  -l]   [wz]
        # [vrr]   [1  -1   l]

        vfl = vx - vy - self.l * wz
        vfr = vx + vy + self.l * wz
        vrl = vx + vy - self.l * wz
        vrr = vx - vy + self.l * wz

        return vfl, vfr, vrl, vrr

    def forward_kinematics(self, vfl: float, vfr: float, vrl: float, vrr: float
                          ) -> Tuple[float, float, float]:
        """
        Calculate robot velocity from wheel velocities

        Args:
            vfl, vfr, vrl, vrr: Wheel velocities (m/s)

        Returns:
            Tuple of (vx, vy, wz)
        """
        vx = (vfl + vfr + vrl + vrr) / 4.0
        vy = (-vfl + vfr + vrl - vrr) / 4.0
        wz = (-vfl + vfr - vrl + vrr) / (4.0 * self.l)

        return vx, vy, wz

    def velocity_to_motor_speeds(self, vx: float, vy: float, wz: float,
                                 max_motor_speed: int = 255
                                 ) -> Tuple[int, int, int, int]:
        """
        Convert velocity commands to motor PWM values

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            wz: Angular velocity (rad/s)
            max_motor_speed: Maximum motor PWM value

        Returns:
            Tuple of motor speeds (fl, fr, rl, rr) in range [-255, 255]
        """
        # Get wheel velocities
        vfl, vfr, vrl, vrr = self.inverse_kinematics(vx, vy, wz)

        # Normalize if any wheel exceeds max speed
        wheel_speeds = np.array([vfl, vfr, vrl, vrr])
        max_wheel_speed = np.max(np.abs(wheel_speeds))

        if max_wheel_speed > self.config.max_speed:
            scale = self.config.max_speed / max_wheel_speed
            wheel_speeds *= scale
            logger.debug(f"Scaled wheel speeds by {scale:.2f}")

        # Convert to motor PWM (-255 to 255)
        motor_speeds = (wheel_speeds / self.config.max_speed * max_motor_speed).astype(int)

        # Apply motor inversions
        if self.config.invert_front_left:
            motor_speeds[0] *= -1
        if self.config.invert_front_right:
            motor_speeds[1] *= -1
        if self.config.invert_rear_left:
            motor_speeds[2] *= -1
        if self.config.invert_rear_right:
            motor_speeds[3] *= -1

        # Clamp to valid range
        motor_speeds = np.clip(motor_speeds, -max_motor_speed, max_motor_speed)

        return tuple(motor_speeds.tolist())

    def move_forward(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Move forward at given speed"""
        return self.velocity_to_motor_speeds(speed, 0, 0)

    def move_backward(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Move backward at given speed"""
        return self.velocity_to_motor_speeds(-speed, 0, 0)

    def strafe_left(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Strafe left at given speed"""
        return self.velocity_to_motor_speeds(0, speed, 0)

    def strafe_right(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Strafe right at given speed"""
        return self.velocity_to_motor_speeds(0, -speed, 0)

    def rotate_left(self, angular_speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Rotate counterclockwise"""
        return self.velocity_to_motor_speeds(0, 0, angular_speed)

    def rotate_right(self, angular_speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Rotate clockwise"""
        return self.velocity_to_motor_speeds(0, 0, -angular_speed)

    def stop(self) -> Tuple[int, int, int, int]:
        """Stop all motors"""
        return (0, 0, 0, 0)

    def move_diagonal(self, speed: float, angle_deg: float) -> Tuple[int, int, int, int]:
        """
        Move in diagonal direction

        Args:
            speed: Movement speed (m/s)
            angle_deg: Direction angle (0=forward, 90=left, 180=back, 270=right)

        Returns:
            Motor speeds
        """
        angle_rad = np.radians(angle_deg)
        vx = speed * np.cos(angle_rad)
        vy = speed * np.sin(angle_rad)

        return self.velocity_to_motor_speeds(vx, vy, 0)

    def holonomic_drive(self, vx: float, vy: float, wz: float) -> Tuple[int, int, int, int]:
        """
        Full holonomic control (move in any direction while rotating)

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            wz: Angular velocity (rad/s)

        Returns:
            Motor speeds
        """
        return self.velocity_to_motor_speeds(vx, vy, wz)

    def get_motion_constraints(self) -> Dict[str, float]:
        """
        Get motion constraints

        Returns:
            Dictionary with max velocities
        """
        return {
            'max_linear_speed': self.config.max_speed,
            'max_angular_speed': self.config.max_speed / self.l,
            'max_acceleration': self.config.max_acceleration
        }


if __name__ == "__main__":
    # Test mechanum drive
    config = MechanumConfig(
        wheel_base=0.30,
        track_width=0.25,
        wheel_radius=0.05,
        max_speed=1.0,
        max_acceleration=0.5
    )

    drive = MechanumDrive(config)

    logger.info("Testing mechanum movements:")

    # Test different movements
    movements = [
        ("Forward", drive.move_forward(0.5)),
        ("Backward", drive.move_backward(0.5)),
        ("Strafe Left", drive.strafe_left(0.5)),
        ("Strafe Right", drive.strafe_right(0.5)),
        ("Rotate Left", drive.rotate_left(0.5)),
        ("Rotate Right", drive.rotate_right(0.5)),
        ("Diagonal 45°", drive.move_diagonal(0.5, 45)),
        ("Stop", drive.stop()),
    ]

    for name, speeds in movements:
        logger.info(f"{name}: FL={speeds[0]:4d}, FR={speeds[1]:4d}, "
                   f"RL={speeds[2]:4d}, RR={speeds[3]:4d}")

    # Test holonomic drive
    logger.info("\nHolonomic (forward + left + rotate):")
    speeds = drive.holonomic_drive(0.5, 0.3, 0.2)
    logger.info(f"FL={speeds[0]:4d}, FR={speeds[1]:4d}, "
               f"RL={speeds[2]:4d}, RR={speeds[3]:4d}")
