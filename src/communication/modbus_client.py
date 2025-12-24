#!/usr/bin/env python3
"""
Modbus RTU Client for Arduino Communication
Handles serial communication via USB using Modbus protocol
"""

import time
from typing import Optional, Dict, List
from loguru import logger
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException


class ModbusArduinoClient:
    """
    Modbus RTU client for communicating with Arduino controller
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200,
                 slave_id: int = 1, timeout: float = 1.0):
        """
        Initialize Modbus client

        Args:
            port: Serial port (e.g., /dev/ttyACM0)
            baudrate: Communication speed
            slave_id: Modbus slave ID
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.client: Optional[ModbusSerialClient] = None
        self.connected = False

        logger.info(f"Initializing Modbus client on {port} @ {baudrate} baud")

    def connect(self) -> bool:
        """
        Connect to Arduino via Modbus RTU

        Returns:
            True if connection successful
        """
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
                logger.success(f"Connected to Arduino on {self.port}")
                return True
            else:
                logger.error(f"Failed to connect to {self.port}")
                return False

        except Exception as e:
            logger.error(f"Connection error: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close Modbus connection"""
        if self.client:
            self.client.close()
            self.connected = False
            logger.info("Modbus connection closed")

    def read_input_registers(self, address: int, count: int = 1) -> Optional[List[int]]:
        """
        Read input registers (sensors data)

        Args:
            address: Starting register address
            count: Number of registers to read

        Returns:
            List of register values or None on error
        """
        if not self.connected:
            logger.warning("Not connected to Arduino")
            return None

        try:
            response = self.client.read_input_registers(
                address=address,
                count=count,
                slave=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus error reading input registers: {response}")
                return None

            return response.registers

        except ModbusException as e:
            logger.error(f"Modbus exception: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error reading registers: {e}")
            return None

    def read_holding_registers(self, address: int, count: int = 1) -> Optional[List[int]]:
        """
        Read holding registers (configuration/control data)

        Args:
            address: Starting register address
            count: Number of registers to read

        Returns:
            List of register values or None on error
        """
        if not self.connected:
            logger.warning("Not connected to Arduino")
            return None

        try:
            response = self.client.read_holding_registers(
                address=address,
                count=count,
                slave=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus error reading holding registers: {response}")
                return None

            return response.registers

        except Exception as e:
            logger.error(f"Error reading holding registers: {e}")
            return None

    def write_register(self, address: int, value: int) -> bool:
        """
        Write single holding register

        Args:
            address: Register address
            value: Value to write (0-65535)

        Returns:
            True if successful
        """
        if not self.connected:
            logger.warning("Not connected to Arduino")
            return False

        try:
            response = self.client.write_register(
                address=address,
                value=value,
                slave=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus error writing register: {response}")
                return False

            return True

        except Exception as e:
            logger.error(f"Error writing register: {e}")
            return False

    def write_registers(self, address: int, values: List[int]) -> bool:
        """
        Write multiple holding registers

        Args:
            address: Starting register address
            values: List of values to write

        Returns:
            True if successful
        """
        if not self.connected:
            logger.warning("Not connected to Arduino")
            return False

        try:
            response = self.client.write_registers(
                address=address,
                values=values,
                slave=self.slave_id
            )

            if response.isError():
                logger.error(f"Modbus error writing registers: {response}")
                return False

            return True

        except Exception as e:
            logger.error(f"Error writing registers: {e}")
            return False

    def read_ultrasonic_sensors(self) -> Optional[Dict[str, float]]:
        """
        Read all 6 ultrasonic sensors

        Returns:
            Dictionary with sensor readings in meters, or None on error
        """
        values = self.read_input_registers(address=0, count=6)

        if values is None:
            return None

        # Convert raw values to meters (assuming Arduino sends cm as uint16)
        sensors = {
            "front_left": values[0] / 100.0,
            "front_center": values[1] / 100.0,
            "front_right": values[2] / 100.0,
            "rear_left": values[3] / 100.0,
            "rear_center": values[4] / 100.0,
            "rear_right": values[5] / 100.0,
        }

        return sensors

    def set_motor_speeds(self, fl: int, fr: int, rl: int, rr: int) -> bool:
        """
        Set motor speeds for all 4 wheels (Mechanum)

        Args:
            fl: Front left motor speed (-255 to 255)
            fr: Front right motor speed (-255 to 255)
            rl: Rear left motor speed (-255 to 255)
            rr: Rear right motor speed (-255 to 255)

        Returns:
            True if successful
        """
        # Convert negative values to unsigned (use two's complement if needed)
        # Or map to 0-511 range where 255 is stop
        values = [
            self._speed_to_register(fl),
            self._speed_to_register(fr),
            self._speed_to_register(rl),
            self._speed_to_register(rr)
        ]

        return self.write_registers(address=100, values=values)

    def emergency_stop(self) -> bool:
        """
        Trigger emergency stop

        Returns:
            True if successful
        """
        logger.warning("Emergency stop triggered!")
        return self.write_register(address=200, value=1)

    def release_emergency_stop(self) -> bool:
        """
        Release emergency stop

        Returns:
            True if successful
        """
        logger.info("Releasing emergency stop")
        return self.write_register(address=200, value=0)

    def set_mode(self, mode: str) -> bool:
        """
        Set system mode

        Args:
            mode: "manual" or "auto"

        Returns:
            True if successful
        """
        mode_value = 1 if mode.lower() == "auto" else 0
        return self.write_register(address=201, value=mode_value)

    @staticmethod
    def _speed_to_register(speed: int) -> int:
        """
        Convert speed (-255 to 255) to register value

        Args:
            speed: Motor speed

        Returns:
            Register value (0-65535)
        """
        # Clamp speed
        speed = max(-255, min(255, speed))

        # Map to 0-511 range where 255 is stop
        # 0-254: reverse, 255: stop, 256-511: forward
        if speed < 0:
            return 255 + speed  # 0 to 254
        elif speed > 0:
            return 255 + speed  # 256 to 510
        else:
            return 255  # Stop

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


if __name__ == "__main__":
    # Test communication
    logger.add("logs/modbus_test.log", rotation="10 MB")

    with ModbusArduinoClient(port="/dev/ttyACM0") as client:
        if client.connected:
            # Test sensor reading
            sensors = client.read_ultrasonic_sensors()
            logger.info(f"Sensor readings: {sensors}")

            # Test motor control
            client.set_motor_speeds(100, 100, 100, 100)
            time.sleep(2)
            client.set_motor_speeds(0, 0, 0, 0)

            logger.success("Communication test completed")
