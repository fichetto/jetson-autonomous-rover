#!/usr/bin/env python3
"""
Simple Modbus Motor Test - Uses same pattern as working datalogger

Tests motor control by sending raw Modbus RTU frames with proper
DTR/RTS handling to prevent Arduino reset.

Based on working code from pi@192.168.1.124:/home/pi/UIC_Profilometer
"""

import sys
import time
import subprocess
import serial

# Configuration
PORT = '/dev/ttyARDUINO'  # Try symlink first, fallback to ttyUSB0
BAUDRATE = 115200
SLAVE_ID = 1

# Modbus registers (from config.h)
HREG_MOTOR_FL_SPEED = 0
HREG_MOTOR_FR_SPEED = 1
HREG_MOTOR_RL_SPEED = 2
HREG_MOTOR_RR_SPEED = 3
MOTOR_STOP = 255


def crc16_modbus(data):
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


class SimpleModbusClient:
    """Simple Modbus RTU client with DTR control - matches working datalogger"""

    def __init__(self, port, baudrate=115200, slave_id=1, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """Open serial connection with DTR disabled"""
        try:
            # Disable hupcl first
            print(f"  Disabling hupcl on {self.port}...")
            result = subprocess.run(['stty', '-F', self.port, '-hupcl'],
                                    capture_output=True, text=True)
            if result.returncode != 0:
                print(f"  Warning: stty returned {result.returncode}: {result.stderr}")

            # Open serial port with DTR/RTS disabled
            print(f"  Opening serial port with DTR=False, RTS=False...")
            self.serial = serial.Serial()
            self.serial.port = self.port
            self.serial.baudrate = self.baudrate
            self.serial.timeout = self.timeout
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.open()

            print(f"  Waiting 2 seconds for Arduino to stabilize...")
            time.sleep(2.0)  # Let Arduino stabilize

            return True
        except Exception as e:
            print(f"  Error connecting: {e}")
            return False

    def close(self):
        """Close serial connection"""
        if self.serial:
            self.serial.close()

    def write_register(self, address, value):
        """Write single holding register (function code 0x06)"""
        if not self.serial:
            return False

        try:
            # Build request frame: slave_id, function, address(2), value(2), crc(2)
            request = bytes([
                self.slave_id,
                0x06,  # Write Single Register
                (address >> 8) & 0xFF,
                address & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF
            ])

            # Add CRC
            crc = crc16_modbus(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Send request
            self.serial.write(request)
            self.serial.flush()

            # Wait for response
            time.sleep(0.05)

            # Read response (echo of request for function 0x06)
            response = self.serial.read(8)
            if len(response) < 8:
                print(f"    Short response: {len(response)} bytes")
                return False

            # Verify response
            if response[0] != self.slave_id or response[1] != 0x06:
                print(f"    Bad response: {response.hex()}")
                return False

            return True

        except Exception as e:
            print(f"    Exception: {e}")
            return False

    def write_registers(self, start_address, values):
        """Write multiple holding registers (function code 0x10)"""
        if not self.serial:
            return False

        try:
            count = len(values)
            byte_count = count * 2

            # Build request frame
            request = bytes([
                self.slave_id,
                0x10,  # Write Multiple Registers
                (start_address >> 8) & 0xFF,
                start_address & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
                byte_count
            ])

            # Add register values
            for val in values:
                request += bytes([(val >> 8) & 0xFF, val & 0xFF])

            # Add CRC
            crc = crc16_modbus(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Send request
            self.serial.write(request)
            self.serial.flush()

            # Wait for response
            time.sleep(0.05)

            # Read response (8 bytes: slave, func, addr(2), count(2), crc(2))
            response = self.serial.read(8)
            if len(response) < 8:
                print(f"    Short response: {len(response)} bytes")
                return False

            # Verify response
            if response[0] != self.slave_id or response[1] != 0x10:
                print(f"    Bad response: {response.hex()}")
                return False

            return True

        except Exception as e:
            print(f"    Exception: {e}")
            return False

    def read_registers(self, address, count):
        """Read holding registers (function code 0x03)"""
        if not self.serial:
            return None

        try:
            # Build request frame
            request = bytes([
                self.slave_id,
                0x03,  # Read Holding Registers
                (address >> 8) & 0xFF,
                address & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF
            ])

            # Add CRC
            crc = crc16_modbus(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Send request
            self.serial.write(request)
            self.serial.flush()

            # Wait for response
            time.sleep(0.05)

            # Read response header
            header = self.serial.read(3)
            if len(header) < 3:
                return None

            if header[0] != self.slave_id or header[1] != 0x03:
                return None

            byte_count = header[2]

            # Read data + CRC
            data = self.serial.read(byte_count + 2)
            if len(data) < byte_count + 2:
                return None

            # Parse registers
            registers = []
            for i in range(0, byte_count, 2):
                reg = (data[i] << 8) | data[i + 1]
                registers.append(reg)

            return registers

        except Exception as e:
            return None


def speed_to_register(speed):
    """Convert -255..+255 speed to 0-510 register value"""
    return speed + 255


def main():
    print("=" * 60)
    print("  MODBUS MOTOR TEST - DTR Control Pattern")
    print("=" * 60)
    print()

    # Try symlink first, fallback to ttyUSB0
    port = PORT
    try:
        import os
        if not os.path.exists(PORT):
            port = '/dev/ttyUSB0'
            print(f"  Symlink {PORT} not found, using {port}")
    except:
        port = '/dev/ttyUSB0'

    print(f"Port: {port}")
    print(f"Baud: {BAUDRATE}")
    print(f"Slave ID: {SLAVE_ID}")
    print()

    # Connect
    print("Connecting...")
    client = SimpleModbusClient(port, BAUDRATE, SLAVE_ID, timeout=0.5)

    if not client.connect():
        print("FAILED to connect!")
        sys.exit(1)

    print("Connected!\n")

    try:
        # Test 1: Read current motor registers
        print("Test 1: Read motor registers...")
        regs = client.read_registers(0, 4)
        if regs:
            print(f"  FL={regs[0]} FR={regs[1]} RL={regs[2]} RR={regs[3]}")
        else:
            print("  Read failed!")

        print()

        # Test 2: Write single motor at 50%
        print("Test 2: Motor FL at 50% for 3 seconds...")
        val = speed_to_register(128)
        print(f"  Writing register 0 = {val} (speed=128)")
        ok = client.write_register(0, val)
        print(f"  Result: {'OK' if ok else 'FAILED'}")

        time.sleep(3)

        # Stop
        print("  Stopping...")
        client.write_register(0, MOTOR_STOP)
        time.sleep(0.5)

        print()

        # Test 3: Write all motors
        print("Test 3: All motors at 30% for 3 seconds...")
        speed = 77  # ~30%
        vals = [speed_to_register(speed)] * 4
        print(f"  Writing registers 0-3 = {vals}")
        ok = client.write_registers(0, vals)
        print(f"  Result: {'OK' if ok else 'FAILED'}")

        time.sleep(3)

        # Stop all
        print("  Stopping all...")
        client.write_registers(0, [MOTOR_STOP] * 4)
        time.sleep(0.5)

        print()

        # Test 4: Interactive
        print("Test 4: Interactive (press Ctrl+C to exit)")
        print("  Commands: 1-4=motor on, s=stop all, q=quit")

        while True:
            try:
                cmd = input("  > ").strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 's':
                    client.write_registers(0, [MOTOR_STOP] * 4)
                    print("    All stopped")
                elif cmd in ['1', '2', '3', '4']:
                    motor = int(cmd) - 1
                    vals = [MOTOR_STOP] * 4
                    vals[motor] = speed_to_register(100)
                    client.write_registers(0, vals)
                    print(f"    Motor {motor} running at 100")
                else:
                    print("    Unknown command")
            except EOFError:
                break

    except KeyboardInterrupt:
        print("\n\nInterrupted!")

    finally:
        # Stop all motors
        print("\nStopping all motors...")
        client.write_registers(0, [MOTOR_STOP] * 4)
        client.close()
        print("Done.")


if __name__ == "__main__":
    main()
