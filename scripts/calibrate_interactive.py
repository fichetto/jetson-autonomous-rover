#!/usr/bin/env python3
"""
Interactive PWM Calibration with Real-Time Control
===================================================
Keys:
  1-4     Select motor
  u/d     Speed up/down selected motor
  SPACE   All motors ON/OFF (40%)
  s       Stop all
  +/-     Adjust offset
  w       Save offsets
  q       Quit
"""

import subprocess
import serial
import time
import sys
import re
import os
import termios
import tty
import select

# Configuration
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SLAVE_ID = 1
CONFIG_FILE = '/home/jetsonnano/autonomous-rover/firmware/clover_arduino/config.h'

MOTOR_NAMES = ['FL', 'FR', 'RL', 'RR']


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


class ModbusMotor:
    def __init__(self, port, baudrate=115200, slave_id=1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.serial = None
        self.speeds = [0, 0, 0, 0]
        self.offsets = [0, 0, 0, 0]

    def connect(self):
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
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def close(self):
        if self.serial:
            self.stop_all()
            self.serial.close()

    def send_speeds(self):
        if not self.serial:
            return False
        try:
            values = [s + 255 for s in self.speeds]
            request = bytes([self.slave_id, 0x10, 0, 0, 0, 4, 8])
            for val in values:
                request += bytes([(val >> 8) & 0xFF, val & 0xFF])
            crc = crc16_modbus(request)
            request += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
            self.serial.reset_input_buffer()
            self.serial.write(request)
            self.serial.flush()
            time.sleep(0.03)
            self.serial.read(8)
            return True
        except:
            return False

    def set_all(self, speed):
        self.speeds = [speed, speed, speed, speed]
        return self.send_speeds()

    def stop_all(self):
        self.speeds = [0, 0, 0, 0]
        return self.send_speeds()

    def load_offsets(self):
        try:
            with open(CONFIG_FILE, 'r') as f:
                content = f.read()
            for i, name in enumerate(MOTOR_NAMES):
                match = re.search(rf'#define\s+PWM_OFFSET_{name}\s+(-?\d+)', content)
                if match:
                    self.offsets[i] = int(match.group(1))
            return True
        except:
            return False

    def save_offsets(self):
        try:
            with open(CONFIG_FILE, 'r') as f:
                content = f.read()
            for i, name in enumerate(MOTOR_NAMES):
                content = re.sub(
                    rf'(#define\s+PWM_OFFSET_{name}\s+)-?\d+',
                    rf'\g<1>{self.offsets[i]}',
                    content
                )
            with open(CONFIG_FILE, 'w') as f:
                f.write(content)
            return True
        except:
            return False


def getch_nonblocking():
    """Get a single character without blocking"""
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None


def print_status(motor, selected):
    """Print current motor status"""
    os.system('clear')
    print("=" * 60)
    print("  CLOVER PWM CALIBRATION - LIFT ROVER!")
    print("=" * 60)
    print()
    print("MOTORS:                          Speed    Offset")
    print("-" * 60)
    for i in range(4):
        sel = ">>>" if i == selected else "   "
        bar = ""
        s = motor.speeds[i]
        if s > 0:
            bar = "[" + "=" * (s // 20) + ">" + " " * (12 - s // 20) + "]"
        elif s < 0:
            bar = "[" + " " * (12 + s // 20) + "<" + "=" * (-s // 20) + "]"
        else:
            bar = "[      |      ]"
        print(f"  {sel} {MOTOR_NAMES[i]}  {bar}   {s:+4d}     {motor.offsets[i]:+3d}")
    print("-" * 60)
    status = "RUNNING" if any(s != 0 for s in motor.speeds) else "STOPPED"
    print(f"  Status: {status}")
    print()
    print("CONTROLS:")
    print("  [1-4] Select    [u/d] Speed ±20    [SPACE] All ON/OFF")
    print("  [+/-] Offset    [s] Stop           [w] Save   [q] Quit")
    print()


def main():
    print("=" * 60)
    print("  CLOVER PWM CALIBRATION")
    print("=" * 60)
    print()
    print("IMPORTANT: Lift the rover! Wheels must be off the ground!")
    print()

    motor = ModbusMotor(PORT, BAUDRATE, SLAVE_ID)

    print("Connecting...")
    if not motor.connect():
        print("FAILED to connect!")
        sys.exit(1)
    print("Connected!")

    motor.load_offsets()
    print(f"Loaded offsets: {motor.offsets}")
    print()
    input("Press ENTER to start...")

    selected = 0
    all_on = False
    base_speed = 100

    # Setup terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print_status(motor, selected)

        while True:
            ch = getch_nonblocking()
            if ch is None:
                continue

            refresh = True

            if ch == 'q':
                break
            elif ch in '1234':
                selected = int(ch) - 1
            elif ch == 'u':  # Up
                motor.speeds[selected] = min(255, motor.speeds[selected] + 20)
                motor.send_speeds()
            elif ch == 'd':  # Down
                motor.speeds[selected] = max(-255, motor.speeds[selected] - 20)
                motor.send_speeds()
            elif ch == ' ':
                if all_on:
                    motor.stop_all()
                    all_on = False
                else:
                    motor.set_all(base_speed)
                    all_on = True
            elif ch == 's':
                motor.stop_all()
                all_on = False
            elif ch in '+=':
                motor.offsets[selected] = min(50, motor.offsets[selected] + 5)
            elif ch in '-_':
                motor.offsets[selected] = max(-50, motor.offsets[selected] - 5)
            elif ch == 'w':
                if motor.save_offsets():
                    print("\n>>> Offsets saved to config.h! Recompile to apply. <<<")
                    time.sleep(1.5)
                else:
                    print("\n>>> ERROR saving! <<<")
                    time.sleep(1)
            else:
                refresh = False

            if refresh:
                print_status(motor, selected)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        motor.close()

    print("\nMotors stopped.")
    print("\nIf you saved offsets, recompile:")
    print("  cd firmware/clover_arduino")
    print("  arduino-cli compile --fqbn arduino:avr:uno .")
    print("  arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno .")


if __name__ == "__main__":
    main()
