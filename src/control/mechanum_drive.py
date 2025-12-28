#!/usr/bin/env python3
"""
CLOVER - Mechanum Wheel Drive Controller
Handles omnidirectional movement kinematics for 4-wheel Mecanum platform.

Hardware:
    - 4x JGB 520 Motors (12V DC, 330 RPM, Hall encoder)
    - Moebius Shield + PCA9685 (I2C) + HR8833 (H-Bridge)
    - Chassis: 300 x 250 mm
"""

import numpy as np
from typing import Tuple, Dict, Optional
from dataclasses import dataclass
from loguru import logger


@dataclass
class MechanumConfig:
    """Mechanum drive configuration for CLOVER robot"""
    wheel_base: float = 0.25        # meters (front-rear distance)
    track_width: float = 0.20       # meters (left-right distance)
    wheel_radius: float = 0.05      # meters (100mm diameter)
    max_speed: float = 1.0          # m/s
    max_acceleration: float = 0.5   # m/s²

    # Motor specifications (JGB 520)
    motor_max_rpm: int = 330        # No-load RPM
    encoder_cpr: int = 11           # Counts per motor revolution
    gear_ratio: int = 30            # Gear reduction

    # Motor inversion flags (adjust based on physical wiring)
    invert_front_left: bool = False
    invert_front_right: bool = True
    invert_rear_left: bool = False
    invert_rear_right: bool = True


@dataclass
class RobotVelocity:
    """Robot velocity state"""
    vx: float = 0.0     # Forward velocity (m/s)
    vy: float = 0.0     # Lateral velocity (m/s, positive = left)
    wz: float = 0.0     # Angular velocity (rad/s, positive = CCW)


class MechanumDrive:
    """
    Mechanum wheel drive controller with inverse/forward kinematics.

    Mecanum wheels allow omnidirectional movement:
    - Forward/Backward
    - Strafe left/right
    - Rotate in place
    - Any combination of the above (holonomic drive)

    Wheel Configuration:
        FL \\   / FR
             X
        RL /   \\ RR

    The rollers on Mecanum wheels are at 45° angles, creating
    forces that allow lateral movement.
    """

    def __init__(self, config: Optional[MechanumConfig] = None):
        """
        Initialize CLOVER Mecanum drive controller.

        Args:
            config: Drive configuration (uses defaults if None)
        """
        self.config = config or MechanumConfig()

        # Kinematic parameters
        # L = half of (wheelbase + track width)
        self.L = (self.config.wheel_base + self.config.track_width) / 2
        self.r = self.config.wheel_radius

        # Current velocity state
        self.velocity = RobotVelocity()

        # Encoder ticks per wheel revolution (after gearbox)
        self.ticks_per_rev = self.config.encoder_cpr * self.config.gear_ratio

        # Calculate max wheel angular velocity from motor RPM
        self.max_wheel_rpm = self.config.motor_max_rpm / self.config.gear_ratio
        self.max_wheel_omega = self.max_wheel_rpm * 2 * np.pi / 60  # rad/s

        logger.info(f"CLOVER Mecanum drive initialized")
        logger.debug(f"  Wheelbase: {self.config.wheel_base*1000:.0f}mm")
        logger.debug(f"  Track width: {self.config.track_width*1000:.0f}mm")
        logger.debug(f"  Max wheel RPM: {self.max_wheel_rpm:.1f}")

    def inverse_kinematics(self, vx: float, vy: float, wz: float
                           ) -> Tuple[float, float, float, float]:
        """
        Calculate wheel velocities from desired robot velocity.

        Mecanum inverse kinematics:
            ω_fl = (1/r) * (vx - vy - L*wz)
            ω_fr = (1/r) * (vx + vy + L*wz)
            ω_rl = (1/r) * (vx + vy - L*wz)
            ω_rr = (1/r) * (vx - vy + L*wz)

        Args:
            vx: Desired forward velocity (m/s)
            vy: Desired lateral velocity (m/s, positive = left)
            wz: Desired angular velocity (rad/s, positive = CCW)

        Returns:
            Tuple of wheel angular velocities (fl, fr, rl, rr) in rad/s
        """
        inv_r = 1.0 / self.r

        omega_fl = inv_r * (vx - vy - self.L * wz)
        omega_fr = inv_r * (vx + vy + self.L * wz)
        omega_rl = inv_r * (vx + vy - self.L * wz)
        omega_rr = inv_r * (vx - vy + self.L * wz)

        return omega_fl, omega_fr, omega_rl, omega_rr

    def forward_kinematics(self, omega_fl: float, omega_fr: float,
                            omega_rl: float, omega_rr: float
                            ) -> Tuple[float, float, float]:
        """
        Calculate robot velocity from wheel angular velocities.

        Args:
            omega_fl, omega_fr, omega_rl, omega_rr: Wheel angular velocities (rad/s)

        Returns:
            Tuple of (vx, vy, wz) robot velocities
        """
        r = self.r
        L = self.L

        vx = (r / 4.0) * (omega_fl + omega_fr + omega_rl + omega_rr)
        vy = (r / 4.0) * (-omega_fl + omega_fr + omega_rl - omega_rr)
        wz = (r / (4.0 * L)) * (-omega_fl + omega_fr - omega_rl + omega_rr)

        return vx, vy, wz

    def velocity_to_motor_speeds(self, vx: float, vy: float, wz: float,
                                  max_pwm: int = 255
                                  ) -> Tuple[int, int, int, int]:
        """
        Convert velocity commands to motor PWM values.

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            wz: Angular velocity (rad/s)
            max_pwm: Maximum motor PWM value (default 255)

        Returns:
            Tuple of motor speeds (fl, fr, rl, rr) in range [-255, 255]
        """
        # Get wheel angular velocities
        omega_fl, omega_fr, omega_rl, omega_rr = self.inverse_kinematics(vx, vy, wz)
        wheel_omegas = np.array([omega_fl, omega_fr, omega_rl, omega_rr])

        # Normalize if any wheel exceeds max angular velocity
        max_omega = np.max(np.abs(wheel_omegas))
        if max_omega > self.max_wheel_omega:
            scale = self.max_wheel_omega / max_omega
            wheel_omegas *= scale
            logger.debug(f"Velocity scaled by {scale:.2f} to limit motor speed")

        # Convert angular velocity to PWM (linear mapping)
        motor_speeds = (wheel_omegas / self.max_wheel_omega * max_pwm).astype(int)

        # Apply motor inversions based on physical wiring
        inversions = np.array([
            -1 if self.config.invert_front_left else 1,
            -1 if self.config.invert_front_right else 1,
            -1 if self.config.invert_rear_left else 1,
            -1 if self.config.invert_rear_right else 1
        ])
        motor_speeds = motor_speeds * inversions

        # Clamp to valid range
        motor_speeds = np.clip(motor_speeds, -max_pwm, max_pwm)

        # Update current velocity state
        self.velocity.vx = vx
        self.velocity.vy = vy
        self.velocity.wz = wz

        return tuple(motor_speeds.tolist())

    def encoder_to_velocity(self, rpm_fl: int, rpm_fr: int,
                             rpm_rl: int, rpm_rr: int
                             ) -> Tuple[float, float, float]:
        """
        Calculate robot velocity from encoder RPM readings.

        Args:
            rpm_fl, rpm_fr, rpm_rl, rpm_rr: Encoder RPM readings

        Returns:
            Tuple of (vx, vy, wz) robot velocities
        """
        # Apply inversions to get correct direction
        inversions = np.array([
            -1 if self.config.invert_front_left else 1,
            -1 if self.config.invert_front_right else 1,
            -1 if self.config.invert_rear_left else 1,
            -1 if self.config.invert_rear_right else 1
        ])

        # Convert RPM to rad/s
        rpm_array = np.array([rpm_fl, rpm_fr, rpm_rl, rpm_rr]) * inversions
        omega_array = rpm_array * 2 * np.pi / 60

        return self.forward_kinematics(*omega_array)

    # =========================================================================
    # Movement Commands
    # =========================================================================

    def move_forward(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Move forward at given speed (m/s)"""
        return self.velocity_to_motor_speeds(speed, 0, 0)

    def move_backward(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Move backward at given speed (m/s)"""
        return self.velocity_to_motor_speeds(-speed, 0, 0)

    def strafe_left(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Strafe left at given speed (m/s)"""
        return self.velocity_to_motor_speeds(0, speed, 0)

    def strafe_right(self, speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Strafe right at given speed (m/s)"""
        return self.velocity_to_motor_speeds(0, -speed, 0)

    def rotate_left(self, angular_speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Rotate counterclockwise (rad/s)"""
        return self.velocity_to_motor_speeds(0, 0, angular_speed)

    def rotate_right(self, angular_speed: float = 0.5) -> Tuple[int, int, int, int]:
        """Rotate clockwise (rad/s)"""
        return self.velocity_to_motor_speeds(0, 0, -angular_speed)

    def stop(self) -> Tuple[int, int, int, int]:
        """Stop all motors"""
        self.velocity = RobotVelocity()
        return (0, 0, 0, 0)

    def move_diagonal(self, speed: float, angle_deg: float) -> Tuple[int, int, int, int]:
        """
        Move in diagonal direction.

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
        Full holonomic control (move in any direction while rotating).

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            wz: Angular velocity (rad/s)

        Returns:
            Motor speeds
        """
        return self.velocity_to_motor_speeds(vx, vy, wz)

    # =========================================================================
    # Utility Methods
    # =========================================================================

    def get_motion_constraints(self) -> Dict[str, float]:
        """Get motion constraints based on motor specifications."""
        max_linear = self.max_wheel_omega * self.r
        max_angular = max_linear / self.L

        return {
            'max_linear_speed': max_linear,
            'max_angular_speed': max_angular,
            'max_acceleration': self.config.max_acceleration,
            'max_motor_rpm': self.max_wheel_rpm
        }

    def get_current_velocity(self) -> RobotVelocity:
        """Get current commanded velocity"""
        return self.velocity

    def rpm_to_linear_velocity(self, rpm: float) -> float:
        """Convert wheel RPM to linear velocity (m/s)"""
        omega = rpm * 2 * np.pi / 60  # rad/s
        return omega * self.r

    def linear_velocity_to_rpm(self, velocity: float) -> float:
        """Convert linear velocity (m/s) to wheel RPM"""
        omega = velocity / self.r  # rad/s
        return omega * 60 / (2 * np.pi)


def create_clover_drive() -> MechanumDrive:
    """Factory function to create CLOVER drive with default config"""
    return MechanumDrive(MechanumConfig())


if __name__ == "__main__":
    # Test CLOVER mechanum drive
    print("=" * 60)
    print("  CLOVER - Mecanum Drive Test")
    print("=" * 60)

    drive = create_clover_drive()

    # Print constraints
    constraints = drive.get_motion_constraints()
    print(f"\nMotion Constraints:")
    print(f"  Max linear speed: {constraints['max_linear_speed']:.2f} m/s")
    print(f"  Max angular speed: {constraints['max_angular_speed']:.2f} rad/s")
    print(f"  Max motor RPM: {constraints['max_motor_rpm']:.1f}")

    print(f"\nTesting movements:")
    print("-" * 60)

    # Test different movements
    movements = [
        ("Forward 0.5 m/s", drive.move_forward(0.5)),
        ("Backward 0.5 m/s", drive.move_backward(0.5)),
        ("Strafe Left 0.5 m/s", drive.strafe_left(0.5)),
        ("Strafe Right 0.5 m/s", drive.strafe_right(0.5)),
        ("Rotate Left 0.5 rad/s", drive.rotate_left(0.5)),
        ("Rotate Right 0.5 rad/s", drive.rotate_right(0.5)),
        ("Diagonal 45°", drive.move_diagonal(0.5, 45)),
        ("Diagonal 135°", drive.move_diagonal(0.5, 135)),
        ("Stop", drive.stop()),
    ]

    for name, speeds in movements:
        print(f"{name:25s} FL={speeds[0]:4d}  FR={speeds[1]:4d}  "
              f"RL={speeds[2]:4d}  RR={speeds[3]:4d}")

    # Test holonomic drive
    print(f"\n{'Holonomic (fwd+left+rot)':25s}", end="")
    speeds = drive.holonomic_drive(0.3, 0.2, 0.3)
    print(f"FL={speeds[0]:4d}  FR={speeds[1]:4d}  "
          f"RL={speeds[2]:4d}  RR={speeds[3]:4d}")

    # Test encoder to velocity
    print("\n" + "=" * 60)
    print("Encoder to Velocity Test:")
    print("-" * 60)
    vx, vy, wz = drive.encoder_to_velocity(100, 100, 100, 100)
    print(f"All wheels @ 100 RPM: vx={vx:.3f} m/s, vy={vy:.3f} m/s, wz={wz:.3f} rad/s")

    print("\n✓ Mecanum drive test completed")
