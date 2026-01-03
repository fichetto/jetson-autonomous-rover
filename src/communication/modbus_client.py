#!/usr/bin/env python3
"""
CLOVER - Modbus RTU Client for Arduino Communication
Handles serial communication via USB using Modbus protocol.

Hardware Architecture:
    Host (Jetson) → USB → Arduino Uno (Modbus Slave)
    Arduino Uno → I2C → PCA9685 (PWM Generator @ 0x40)
    PCA9685 → HR8833 (H-Bridge) → JGB 520 Motors

The Arduino handles the I2C communication with PCA9685 internally.
This client only communicates via Modbus RTU over USB serial.
"""

import time
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from loguru import logger

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException
except ImportError:
    logger.warning("pymodbus not installed. Run: pip install pymodbus")
    ModbusSerialClient = None
    ModbusException = Exception


@dataclass
class MotorState:
    """Motor state data"""
    speed_command: int = 0      # -255 to 255
    encoder_count: int = 0      # Total encoder counts
    encoder_rpm: int = 0        # Current RPM from encoder


@dataclass
class SystemState:
    """System state data"""
    emergency_stop: bool = False
    mode: str = "manual"        # "manual" or "auto"
    battery_voltage: float = 0.0  # Volts


class ModbusRegisterMap:
    """
    Modbus register addresses for CLOVER robot.
    Must match Arduino firmware implementation (config.h).

    Arduino uses ModbusRTU library with separate arrays:
    - Holding registers: indices 0-19
    - Input registers: indices 0-24
    """
    # Motor Speed Commands (Holding Registers - Read/Write)
    # Match Arduino: HREG_MOTOR_FL_SPEED = 0, etc.
    MOTOR_FL_SPEED = 0      # Holding[0] - M1 Front Left
    MOTOR_FR_SPEED = 1      # Holding[1] - M2 Front Right
    MOTOR_RL_SPEED = 2      # Holding[2] - M3 Rear Left
    MOTOR_RR_SPEED = 3      # Holding[3] - M4 Rear Right

    # Encoder Counts (Input Registers - Read Only)
    # Match Arduino: IREG_ENCODER_M1_COUNT = 0, etc.
    ENCODER_M1_COUNT = 0    # Input[0]
    ENCODER_M2_COUNT = 1    # Input[1]
    ENCODER_M3_COUNT = 2    # Input[2]
    ENCODER_M4_COUNT = 3    # Input[3]

    # Encoder Speed RPM (Input Registers - Read Only)
    # Match Arduino: IREG_ENCODER_M1_SPEED = 10, etc.
    ENCODER_M1_SPEED = 10   # Input[10]
    ENCODER_M2_SPEED = 11   # Input[11]
    ENCODER_M3_SPEED = 12   # Input[12]
    ENCODER_M4_SPEED = 13   # Input[13]

    # System Status (Holding Registers - Read/Write)
    # Match Arduino: HREG_EMERGENCY_STOP = 10, etc.
    EMERGENCY_STOP = 10     # Holding[10]: 0=normal, 1=stop
    SYSTEM_MODE = 11        # Holding[11]: 0=manual, 1=auto

    # Battery Voltage (Input Register - Read Only)
    # Match Arduino: IREG_BATTERY_VOLTAGE = 20
    BATTERY_VOLTAGE = 20    # Input[20]: mV (divide by 1000 for V)

    # Ultrasonic Sensors (Input Registers - Read Only)
    # Reserved for future expansion
    ULTRASONIC_FL = 400
    ULTRASONIC_FC = 401
    ULTRASONIC_FR = 402
    ULTRASONIC_RL = 403
    ULTRASONIC_RC = 404
    ULTRASONIC_RR = 405


class CloverModbusClient:
    """
    Modbus RTU client for CLOVER robot communication with Arduino.

    The Arduino acts as Modbus slave and controls:
    - PCA9685 PWM generator via I2C for motor speed
    - HR8833 H-Bridge for motor direction
    - Encoder reading (D2/D3 for M1/M2, polling for M3/M4)
    - Battery voltage monitoring
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200,
                 slave_id: int = 1, timeout: float = 1.0):
        """
        Initialize Modbus client for CLOVER robot.

        Args:
            port: Serial port (e.g., /dev/ttyACM0)
            baudrate: Communication speed (default 115200)
            slave_id: Modbus slave ID (default 1)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.reg = ModbusRegisterMap()

        # Motor states
        self.motors = {
            'FL': MotorState(),
            'FR': MotorState(),
            'RL': MotorState(),
            'RR': MotorState()
        }

        # System state
        self.system = SystemState()

        logger.info(f"CLOVER Modbus client initialized: {port} @ {baudrate} baud")

    @property
    def is_connected(self) -> bool:
        """Check if client is connected (property for compatibility)"""
        return self.connected

    def connect(self) -> bool:
        """
        Connect to Arduino via Modbus RTU.

        Returns:
            True if connection successful
        """
        if ModbusSerialClient is None:
            logger.error("pymodbus not installed")
            return False

        try:
            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                stopbits=1,
                bytesize=8,
                parity='N'
            )

            if self.client.connect():
                self.connected = True
                logger.success(f"CLOVER connected on {self.port}")

                # Read initial state
                self._read_system_state()
                return True
            else:
                logger.error(f"Failed to connect to {self.port}")
                return False

        except Exception as e:
            logger.error(f"Connection error: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close Modbus connection and stop motors"""
        if self.connected:
            # Stop all motors before disconnecting
            self.stop_all_motors()

        if self.client:
            self.client.close()
            self.connected = False
            logger.info("CLOVER Modbus connection closed")

    def _read_input_registers(self, address: int, count: int = 1) -> Optional[List[int]]:
        """Read input registers (read-only data from Arduino)"""
        if not self.connected or not self.client:
            return None

        try:
            response = self.client.read_input_registers(
                address=address,
                count=count,
                device_id=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus read error @ {address}: {response}")
                return None

            return response.registers

        except Exception as e:
            logger.error(f"Read input registers error: {e}")
            return None

    def _read_holding_registers(self, address: int, count: int = 1) -> Optional[List[int]]:
        """Read holding registers (read/write data)"""
        if not self.connected or not self.client:
            return None

        try:
            response = self.client.read_holding_registers(
                address=address,
                count=count,
                device_id=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus read holding error @ {address}: {response}")
                return None

            return response.registers

        except Exception as e:
            logger.error(f"Read holding registers error: {e}")
            return None

    def _write_register(self, address: int, value: int) -> bool:
        """Write single holding register"""
        if not self.connected or not self.client:
            return False

        try:
            response = self.client.write_register(
                address=address,
                value=value,
                device_id=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus write error @ {address}: {response}")
                return False

            return True

        except Exception as e:
            logger.error(f"Write register error: {e}")
            return False

    def _write_registers(self, address: int, values: List[int]) -> bool:
        """Write multiple holding registers"""
        if not self.connected or not self.client:
            return False

        try:
            response = self.client.write_registers(
                address=address,
                values=values,
                device_id=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus write multiple error @ {address}: {response}")
                return False

            return True

        except Exception as e:
            logger.error(f"Write registers error: {e}")
            return False

    # =========================================================================
    # Motor Control
    # =========================================================================

    def set_motor_speeds(self, fl: int, fr: int, rl: int, rr: int) -> bool:
        """
        Set speed for all 4 Mecanum wheels.

        Args:
            fl: Front Left motor speed (-255 to 255)
            fr: Front Right motor speed (-255 to 255)
            rl: Rear Left motor speed (-255 to 255)
            rr: Rear Right motor speed (-255 to 255)

        Returns:
            True if successful

        Note:
            Positive = forward, Negative = reverse
            Arduino will handle PCA9685/HR8833 control
        """
        # Convert signed speeds to register values
        values = [
            self._speed_to_register(fl),
            self._speed_to_register(fr),
            self._speed_to_register(rl),
            self._speed_to_register(rr)
        ]

        success = self._write_registers(self.reg.MOTOR_FL_SPEED, values)

        if success:
            self.motors['FL'].speed_command = fl
            self.motors['FR'].speed_command = fr
            self.motors['RL'].speed_command = rl
            self.motors['RR'].speed_command = rr

        return success

    def stop_all_motors(self) -> bool:
        """Stop all motors immediately"""
        logger.info("Stopping all motors")
        return self.set_motor_speeds(0, 0, 0, 0)

    def read_encoder_counts(self) -> Optional[Dict[str, int]]:
        """
        Read encoder counts from all 4 motors.

        Returns:
            Dictionary with encoder counts or None on error
        """
        values = self._read_input_registers(self.reg.ENCODER_M1_COUNT, 4)

        if values is None:
            return None

        # Handle signed values (16-bit two's complement)
        counts = {
            'FL': self._register_to_signed(values[0]),
            'FR': self._register_to_signed(values[1]),
            'RL': self._register_to_signed(values[2]),
            'RR': self._register_to_signed(values[3])
        }

        # Update motor states
        for key, count in counts.items():
            self.motors[key].encoder_count = count

        return counts

    def read_encoder_speeds(self) -> Optional[Dict[str, int]]:
        """
        Read encoder speeds (RPM) from all 4 motors.

        Returns:
            Dictionary with speeds in RPM or None on error
        """
        values = self._read_input_registers(self.reg.ENCODER_M1_SPEED, 4)

        if values is None:
            return None

        speeds = {
            'FL': self._register_to_signed(values[0]),
            'FR': self._register_to_signed(values[1]),
            'RL': self._register_to_signed(values[2]),
            'RR': self._register_to_signed(values[3])
        }

        # Update motor states
        for key, rpm in speeds.items():
            self.motors[key].encoder_rpm = rpm

        return speeds

    # =========================================================================
    # System Control
    # =========================================================================

    def emergency_stop(self) -> bool:
        """Trigger emergency stop - stops all motors immediately"""
        logger.warning("EMERGENCY STOP TRIGGERED!")
        self.system.emergency_stop = True
        return self._write_register(self.reg.EMERGENCY_STOP, 1)

    def release_emergency_stop(self) -> bool:
        """Release emergency stop"""
        logger.info("Releasing emergency stop")
        self.system.emergency_stop = False
        return self._write_register(self.reg.EMERGENCY_STOP, 0)

    def set_mode(self, mode: str) -> bool:
        """
        Set system mode.

        Args:
            mode: "manual" or "auto"

        Returns:
            True if successful
        """
        mode_value = 1 if mode.lower() == "auto" else 0
        success = self._write_register(self.reg.SYSTEM_MODE, mode_value)

        if success:
            self.system.mode = mode.lower()

        return success

    def read_battery_voltage(self) -> Optional[float]:
        """
        Read battery voltage.

        Returns:
            Voltage in Volts or None on error

        Battery specs (LiPo 3S2P):
            - Nominal: 11.1V
            - Max: 12.6V
            - Min (cutoff): 9.0V
            - Warning: 10.0V
        """
        values = self._read_input_registers(self.reg.BATTERY_VOLTAGE, 1)

        if values is None:
            return None

        # Arduino sends mV, convert to V
        voltage = values[0] / 1000.0
        self.system.battery_voltage = voltage

        # Log warnings (temporarily disabled for testing)
        # TODO: Re-enable battery warnings once voltage divider is calibrated
        # if voltage < 9.0:
        #     logger.critical(f"BATTERY CRITICAL: {voltage:.2f}V - SHUTDOWN REQUIRED!")
        # elif voltage < 10.0:
        #     logger.warning(f"Battery low: {voltage:.2f}V")

        return voltage

    def _read_system_state(self):
        """Read current system state from Arduino"""
        try:
            # Read emergency stop and mode
            values = self._read_holding_registers(self.reg.EMERGENCY_STOP, 2)
            if values:
                self.system.emergency_stop = bool(values[0])
                self.system.mode = "auto" if values[1] == 1 else "manual"

            # Read battery voltage
            self.read_battery_voltage()

        except Exception as e:
            logger.error(f"Error reading system state: {e}")

    # =========================================================================
    # Ultrasonic Sensors (Reserved for expansion)
    # =========================================================================

    def read_ultrasonic_sensors(self) -> Optional[Dict[str, float]]:
        """
        Read all 6 ultrasonic sensors.

        Returns:
            Dictionary with sensor readings in meters, or None on error

        Note: Currently not connected on hardware.
        """
        values = self._read_input_registers(self.reg.ULTRASONIC_FL, 6)

        if values is None:
            return None

        # Convert raw values to meters (Arduino sends cm as uint16)
        sensors = {
            "front_left": values[0] / 100.0,
            "front_center": values[1] / 100.0,
            "front_right": values[2] / 100.0,
            "rear_left": values[3] / 100.0,
            "rear_center": values[4] / 100.0,
            "rear_right": values[5] / 100.0,
        }

        return sensors

    # =========================================================================
    # Utility Methods
    # =========================================================================

    @staticmethod
    def _speed_to_register(speed: int) -> int:
        """
        Convert signed speed (-255 to 255) to unsigned register value.

        Mapping:
            -255 to -1  → 0 to 254 (reverse)
            0           → 255 (stop)
            1 to 255    → 256 to 510 (forward)

        Args:
            speed: Motor speed (-255 to 255)

        Returns:
            Unsigned register value (0-510)
        """
        speed = max(-255, min(255, speed))
        return 255 + speed

    @staticmethod
    def _register_to_speed(value: int) -> int:
        """Convert register value back to signed speed"""
        return value - 255

    @staticmethod
    def _register_to_signed(value: int) -> int:
        """Convert 16-bit unsigned register to signed value"""
        if value >= 32768:
            return value - 65536
        return value

    def get_status(self) -> Dict:
        """Get current robot status"""
        return {
            'connected': self.connected,
            'emergency_stop': self.system.emergency_stop,
            'mode': self.system.mode,
            'battery_voltage': self.system.battery_voltage,
            'motors': {
                key: {
                    'speed_command': m.speed_command,
                    'encoder_count': m.encoder_count,
                    'encoder_rpm': m.encoder_rpm
                }
                for key, m in self.motors.items()
            }
        }

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# Backwards compatibility alias
ModbusArduinoClient = CloverModbusClient


if __name__ == "__main__":
    # Test communication
    logger.add("logs/modbus_test.log", rotation="10 MB")

    print("=" * 60)
    print("  CLOVER - Modbus Communication Test")
    print("=" * 60)

    with CloverModbusClient(port="/dev/ttyACM0") as client:
        if client.connected:
            print("\n✓ Connected to Arduino")

            # Read battery voltage
            voltage = client.read_battery_voltage()
            if voltage:
                print(f"✓ Battery: {voltage:.2f}V")

            # Read encoder counts
            counts = client.read_encoder_counts()
            if counts:
                print(f"✓ Encoders: {counts}")

            # Test motor control (brief)
            print("\n  Testing motors (50% forward for 1 second)...")
            client.set_motor_speeds(128, 128, 128, 128)
            time.sleep(1)
            client.stop_all_motors()
            print("✓ Motor test complete")

            # Show status
            print(f"\n  Status: {client.get_status()}")

            logger.success("CLOVER communication test completed")
        else:
            print("✗ Failed to connect to Arduino")
            print("  Check that Arduino is connected to /dev/ttyACM0")
