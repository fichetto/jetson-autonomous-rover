#!/usr/bin/env python3
"""
Motor Speed Test - Non-Interactive
===================================
Runs motors to help identify which ones are faster/slower.

Usage:
  python3 test_motor_speeds.py all     # All motors at 40% for 5 sec
  python3 test_motor_speeds.py seq     # Each motor individually 2 sec
  python3 test_motor_speeds.py [1-4]   # Single motor (1=FL, 2=FR, 3=RL, 4=RR)
"""

import subprocess
import serial
import time
import sys
import os

# Configuration
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SLAVE_ID = 1
BASE_SPEED = 100  # ~40%

MOTOR_NAMES = ['FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)']


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class ModbusClient:
    def __init__(self, port, baudrate=115200, slave_id=1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.serial = None

    def connect(self):
        try:
            subprocess.run(['stty', '-F', self.port, '-hupcl'], capture_output=True)
            self.serial = serial.Serial()
            self.serial.port = self.port
            self.serial.baudrate = self.baudrate
            self.serial.timeout = 0.5
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.open()
            time.sleep(2)
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def close(self):
        if self.serial:
            self.serial.close()

    def set_motors(self, speeds):
        """Set motor speeds (-255 to +255)"""
        if not self.serial:
            return False
        try:
            values = [s + 255 for s in speeds]
            count = len(values)
            request = bytes([
                self.slave_id, 0x10,
                0, 0,  # start address
                (count >> 8) & 0xFF, count & 0xFF,
                count * 2
            ])
            for val in values:
                request += bytes([(val >> 8) & 0xFF, val & 0xFF])
            crc = crc16_modbus(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
            self.serial.reset_input_buffer()
            self.serial.write(request)
            self.serial.flush()
            time.sleep(0.05)
            self.serial.read(8)
            return True
        except:
            return False

    def stop_all(self):
        return self.set_motors([0, 0, 0, 0])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    mode = sys.argv[1].lower()

    print("=" * 50)
    print("  MOTOR SPEED TEST")
    print("=" * 50)
    print("\nIMPORTANTE: Alza il rover! Ruote sollevate da terra!\n")
    print("Connecting...")

    client = ModbusClient(PORT, BAUDRATE, SLAVE_ID)
    if not client.connect():
        print("Connection failed!")
        sys.exit(1)
    print("Connected!\n")

    try:
        if mode == 'all':
            print(f"All motors at {BASE_SPEED/255*100:.0f}% for 5 seconds...")
            print("Observe which wheel spins faster/slower.\n")
            client.set_motors([BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED])
            for i in range(5, 0, -1):
                print(f"  {i}...")
                time.sleep(1)
            client.stop_all()
            print("\nSTOP")

        elif mode == 'seq':
            print("Sequential test - each motor for 2 seconds:\n")
            for i in range(4):
                print(f"  {MOTOR_NAMES[i]}...")
                speeds = [0, 0, 0, 0]
                speeds[i] = BASE_SPEED
                client.set_motors(speeds)
                time.sleep(2)
            client.stop_all()
            print("\nSTOP")

        elif mode in ['1', '2', '3', '4']:
            motor = int(mode) - 1
            print(f"{MOTOR_NAMES[motor]} at {BASE_SPEED/255*100:.0f}% for 3 seconds...")
            speeds = [0, 0, 0, 0]
            speeds[motor] = BASE_SPEED
            client.set_motors(speeds)
            time.sleep(3)
            client.stop_all()
            print("STOP")

        else:
            print(f"Unknown mode: {mode}")
            print("Use: all, seq, or 1-4")

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        client.stop_all()
        client.close()
        print("\nMotors stopped. Done.")


if __name__ == "__main__":
    main()
