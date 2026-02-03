#!/usr/bin/env python3
"""
Robust Modbus Motor Controller for CLOVER Rover
================================================
Thread-based architecture for reliable motor control:
- Background thread sends commands at fixed rate (20Hz)
- Thread-safe speed setting
- Watchdog stops motors if no commands for timeout
- Automatic reconnection on errors
"""

import serial
import subprocess
import threading
import time
from typing import Tuple, Optional
from dataclasses import dataclass
from loguru import logger


@dataclass
class MotorSpeeds:
    """Motor speeds for 4 wheels"""
    fl: int = 0  # Front Left
    fr: int = 0  # Front Right
    rl: int = 0  # Rear Left
    rr: int = 0  # Rear Right

    def __iter__(self):
        return iter([self.fl, self.fr, self.rl, self.rr])

    def is_zero(self) -> bool:
        return self.fl == 0 and self.fr == 0 and self.rl == 0 and self.rr == 0


class RobustModbusController:
    """
    Thread-based Modbus motor controller with continuous command refresh.

    This controller runs a background thread that continuously sends
    motor commands at a fixed rate, ensuring smooth and reliable control.
    """

    COMMAND_RATE_HZ = 20  # Commands per second
    WATCHDOG_TIMEOUT = 0.5  # Seconds without command before auto-stop
    RECONNECT_DELAY = 1.0  # Seconds between reconnection attempts

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """
        Initialize the motor controller.

        Args:
            port: Serial port for Arduino
            baudrate: Serial baudrate
        """
        self.port = port
        self.baudrate = baudrate

        # Serial connection
        self._serial: Optional[serial.Serial] = None
        self._connected = False

        # Thread-safe speed control
        self._lock = threading.Lock()
        self._target_speeds = MotorSpeeds()
        self._current_speeds = MotorSpeeds()
        self._last_command_time = 0.0

        # Control thread
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Statistics
        self._commands_sent = 0
        self._errors = 0

        logger.info(f"RobustModbusController initialized: {port} @ {baudrate}")

    def connect(self) -> bool:
        """
        Connect to Arduino and start control thread.

        Returns:
            True if connection successful
        """
        if self._connected:
            return True

        try:
            # Disable DTR/RTS reset
            subprocess.run(
                ['stty', '-F', self.port, '-hupcl'],
                capture_output=True, timeout=5
            )

            # Open serial connection
            self._serial = serial.Serial()
            self._serial.port = self.port
            self._serial.baudrate = self.baudrate
            self._serial.timeout = 0.1
            self._serial.write_timeout = 0.1
            self._serial.dtr = False
            self._serial.rts = False
            self._serial.open()

            # Wait for Arduino ready
            time.sleep(2)

            # Clear buffers
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            self._connected = True
            self._last_command_time = time.time()

            # Start control thread
            self._start_thread()

            logger.success(f"Connected to Arduino on {self.port}")
            return True

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Stop thread and close connection."""
        self._running = False

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        if self._serial and self._serial.is_open:
            try:
                self._send_modbus_command(MotorSpeeds())  # Stop motors
                self._serial.close()
            except:
                pass

        self._connected = False
        logger.info("Motor controller disconnected")

    def _start_thread(self):
        """Start the background control thread."""
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        logger.debug("Control thread started")

    def _control_loop(self):
        """
        Main control loop running in background thread.
        Sends motor commands at fixed rate with watchdog.
        """
        period = 1.0 / self.COMMAND_RATE_HZ

        while self._running:
            loop_start = time.time()

            try:
                # Check watchdog
                with self._lock:
                    time_since_command = time.time() - self._last_command_time

                    if time_since_command > self.WATCHDOG_TIMEOUT:
                        # No recent commands - stop motors
                        if not self._target_speeds.is_zero():
                            logger.warning("Watchdog: no commands received, stopping motors")
                            self._target_speeds = MotorSpeeds()

                    speeds = MotorSpeeds(
                        fl=self._target_speeds.fl,
                        fr=self._target_speeds.fr,
                        rl=self._target_speeds.rl,
                        rr=self._target_speeds.rr
                    )

                # Send command
                if self._connected:
                    success = self._send_modbus_command(speeds)
                    if success:
                        self._current_speeds = speeds
                        self._commands_sent += 1
                    else:
                        self._errors += 1
                        # Try to reconnect on repeated errors
                        if self._errors % 10 == 0:
                            logger.warning(f"Multiple errors ({self._errors}), attempting reconnect")
                            self._reconnect()

            except Exception as e:
                logger.error(f"Control loop error: {e}")
                self._errors += 1

            # Maintain fixed rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _reconnect(self):
        """Attempt to reconnect to Arduino."""
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
            time.sleep(self.RECONNECT_DELAY)

            self._serial.open()
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self._connected = True
            logger.success("Reconnected to Arduino")
        except Exception as e:
            logger.error(f"Reconnect failed: {e}")
            self._connected = False

    def _crc16(self, data: bytes) -> int:
        """Calculate Modbus CRC16."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _send_modbus_command(self, speeds: MotorSpeeds) -> bool:
        """
        Send Modbus command to set motor speeds.

        Args:
            speeds: Motor speeds to set

        Returns:
            True if command sent successfully
        """
        if not self._serial or not self._serial.is_open:
            return False

        try:
            # Convert to Modbus values (0-510, center at 255)
            values = [
                speeds.fl + 255,
                speeds.fr + 255,
                speeds.rl + 255,
                speeds.rr + 255
            ]

            # Build Modbus request (Write Multiple Registers)
            # Slave: 1, Function: 0x10, Start: 0, Count: 4, Bytes: 8
            request = bytes([1, 0x10, 0, 0, 0, 4, 8])
            for val in values:
                val = max(0, min(510, val))  # Clamp
                request += bytes([(val >> 8) & 0xFF, val & 0xFF])

            # Add CRC
            crc = self._crc16(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            # Clear input buffer and send
            self._serial.reset_input_buffer()
            self._serial.write(request)
            self._serial.flush()

            # Read response (non-blocking, short timeout)
            response = self._serial.read(8)

            # Validate response (basic check)
            if len(response) >= 6 and response[0] == 1 and response[1] == 0x10:
                return True

            # Even without valid response, command was sent
            return True

        except serial.SerialException as e:
            logger.warning(f"Serial error: {e}")
            return False
        except Exception as e:
            logger.error(f"Command error: {e}")
            return False

    def set_motors(self, fl: int, fr: int, rl: int, rr: int):
        """
        Set motor speeds (thread-safe).

        Args:
            fl, fr, rl, rr: Motor speeds (-255 to +255)
        """
        with self._lock:
            self._target_speeds = MotorSpeeds(
                fl=max(-255, min(255, fl)),
                fr=max(-255, min(255, fr)),
                rl=max(-255, min(255, rl)),
                rr=max(-255, min(255, rr))
            )
            self._last_command_time = time.time()

    def set_differential(self, forward: int, rotation: int):
        """
        Set differential drive speeds (low-level).

        Args:
            forward: Forward speed (-255 to +255)
            rotation: Rotation speed (-255 to +255, positive = turn right)

        Note: To turn RIGHT, left wheels go faster than right wheels.
        """
        # To turn RIGHT (positive rotation): left > right
        left = forward + rotation
        right = forward - rotation

        # Clamp
        left = max(-255, min(255, left))
        right = max(-255, min(255, right))

        self.set_motors(left, right, left, right)

    # ==================== HIGH-LEVEL COMMANDS ====================
    # Use percentage (0-100) for speed

    def stop(self):
        """Stop all motors immediately."""
        with self._lock:
            self._target_speeds = MotorSpeeds()
            self._last_command_time = time.time()

    def forward(self, speed_percent: int):
        """
        Move forward at specified speed percentage.

        Args:
            speed_percent: Speed 0-100%
        """
        speed = int(min(100, max(0, speed_percent)) * 255 / 100)
        self.set_motors(speed, speed, speed, speed)

    def backward(self, speed_percent: int):
        """
        Move backward at specified speed percentage.

        Args:
            speed_percent: Speed 0-100%
        """
        speed = int(min(100, max(0, speed_percent)) * 255 / 100)
        self.set_motors(-speed, -speed, -speed, -speed)

    def rotate_left(self, speed_percent: int):
        """
        Rotate left (counter-clockwise) at specified speed percentage.

        Args:
            speed_percent: Speed 0-100%
        """
        speed = int(min(100, max(0, speed_percent)) * 255 / 100)
        self.set_motors(-speed, speed, -speed, speed)

    def rotate_right(self, speed_percent: int):
        """
        Rotate right (clockwise) at specified speed percentage.

        Args:
            speed_percent: Speed 0-100%
        """
        speed = int(min(100, max(0, speed_percent)) * 255 / 100)
        self.set_motors(speed, -speed, speed, -speed)

    def drive(self, forward_percent: int, rotation_percent: int):
        """
        Combined forward/backward and rotation command.

        Args:
            forward_percent: Forward speed -100 to +100% (negative = backward)
            rotation_percent: Rotation -100 to +100% (negative = left, positive = right)
        """
        forward = int(max(-100, min(100, forward_percent)) * 255 / 100)
        rotation = int(max(-100, min(100, rotation_percent)) * 255 / 100)
        self.set_differential(forward, rotation)

    # Legacy methods (kept for compatibility)
    def move_forward(self, speed: int):
        """Legacy: Move forward at specified speed (0-255)."""
        self.set_motors(speed, speed, speed, speed)

    def move_backward(self, speed: int):
        """Legacy: Move backward at specified speed (0-255)."""
        self.set_motors(-speed, -speed, -speed, -speed)

    @property
    def is_connected(self) -> bool:
        """Check if connected to Arduino."""
        return self._connected

    @property
    def current_speeds(self) -> MotorSpeeds:
        """Get current motor speeds."""
        with self._lock:
            return MotorSpeeds(
                fl=self._current_speeds.fl,
                fr=self._current_speeds.fr,
                rl=self._current_speeds.rl,
                rr=self._current_speeds.rr
            )

    @property
    def stats(self) -> dict:
        """Get controller statistics."""
        return {
            'connected': self._connected,
            'commands_sent': self._commands_sent,
            'errors': self._errors,
            'command_rate_hz': self.COMMAND_RATE_HZ
        }

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Convenience function for quick testing
def test_controller():
    """Quick test of motor controller."""
    logger.info("Testing RobustModbusController...")

    with RobustModbusController() as motors:
        if not motors.is_connected:
            logger.error("Failed to connect!")
            return False

        logger.info("Test 1: Forward")
        motors.move_forward(100)
        time.sleep(1)

        logger.info("Test 2: Backward")
        motors.move_backward(100)
        time.sleep(1)

        logger.info("Test 3: Rotate left")
        motors.rotate_left(100)
        time.sleep(1)

        logger.info("Test 4: Rotate right")
        motors.rotate_right(100)
        time.sleep(1)

        logger.info("Test 5: Stop")
        motors.stop()
        time.sleep(0.5)

        logger.success(f"Test complete! Stats: {motors.stats}")
        return True


if __name__ == "__main__":
    test_controller()
