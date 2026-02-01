#!/usr/bin/env python3
"""
Test script for PID closed-loop motor control.
Tests the new velocity control system on the CLOVER rover.
"""

import time
import sys
from pymodbus.client import ModbusSerialClient

# Modbus registers
HREG_MOTOR_FL_SPEED = 0
HREG_MOTOR_FR_SPEED = 1
HREG_MOTOR_RL_SPEED = 2
HREG_MOTOR_RR_SPEED = 3
HREG_EMERGENCY_STOP = 10
HREG_TARGET_RPM_FL = 20
HREG_TARGET_RPM_FR = 21
HREG_TARGET_RPM_RL = 22
HREG_TARGET_RPM_RR = 23
HREG_PID_ENABLE = 24

IREG_ENCODER_M1_SPEED = 10
IREG_ENCODER_M2_SPEED = 11
IREG_ENCODER_M3_SPEED = 12
IREG_ENCODER_M4_SPEED = 13

RPM_OFFSET = 200
RPM_SPEED_OFFSET = 32768

MOTOR_NAMES = ['FL', 'FR', 'RL', 'RR']


def rpm_to_register(rpm: int) -> int:
    """Convert signed RPM (-200 to +200) to register value (0 to 400)."""
    return rpm + RPM_OFFSET


def register_to_rpm(reg: int) -> int:
    """Convert register value to signed RPM."""
    return reg - RPM_SPEED_OFFSET


class PIDTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.client = ModbusSerialClient(port=port, baudrate=baudrate)
        self.slave_id = 1

    def connect(self) -> bool:
        if not self.client.connect():
            print("ERROR: Cannot connect to Arduino")
            return False
        print(f"Connected to Arduino on {self.client.comm_params.host}")
        return True

    def disconnect(self):
        self.client.close()
        print("Disconnected")

    def enable_pid(self):
        """Enable PID closed-loop control."""
        self.client.write_register(HREG_PID_ENABLE, 1, device_id=self.slave_id)
        print("PID enabled (closed-loop mode)")

    def disable_pid(self):
        """Disable PID, return to open-loop mode."""
        self.client.write_register(HREG_PID_ENABLE, 0, device_id=self.slave_id)
        print("PID disabled (open-loop mode)")

    def set_target_rpm(self, fl: int, fr: int, rl: int, rr: int):
        """Set target RPM for all motors (-200 to +200)."""
        values = [rpm_to_register(fl), rpm_to_register(fr),
                  rpm_to_register(rl), rpm_to_register(rr)]
        self.client.write_registers(HREG_TARGET_RPM_FL, values, device_id=self.slave_id)
        print(f"Target RPM set: FL={fl}, FR={fr}, RL={rl}, RR={rr}")

    def set_all_rpm(self, rpm: int):
        """Set same target RPM for all motors."""
        self.set_target_rpm(rpm, rpm, rpm, rpm)

    def read_measured_rpm(self) -> list:
        """Read measured RPM from encoders."""
        result = self.client.read_input_registers(IREG_ENCODER_M1_SPEED, count=4, device_id=self.slave_id)
        if result.isError():
            print("ERROR reading encoder speeds")
            return [0, 0, 0, 0]
        return [register_to_rpm(v) for v in result.registers]

    def stop_all(self):
        """Stop all motors (set target to 0 RPM)."""
        self.set_all_rpm(0)
        print("Motors stopped")

    def emergency_stop(self):
        """Trigger emergency stop."""
        self.client.write_register(HREG_EMERGENCY_STOP, 1, device_id=self.slave_id)
        print("EMERGENCY STOP!")

    def clear_emergency(self):
        """Clear emergency stop."""
        self.client.write_register(HREG_EMERGENCY_STOP, 0, device_id=self.slave_id)
        print("Emergency cleared")

    def print_status(self):
        """Print current motor status."""
        rpm = self.read_measured_rpm()
        print(f"  Measured RPM: FL={rpm[0]:4d}, FR={rpm[1]:4d}, RL={rpm[2]:4d}, RR={rpm[3]:4d}")

    def monitor(self, duration: float = 5.0, interval: float = 0.2):
        """Monitor motor speeds for a duration."""
        print(f"\nMonitoring for {duration}s...")
        start = time.time()
        while time.time() - start < duration:
            self.print_status()
            time.sleep(interval)


def test_step_response(tester: PIDTester, target_rpm: int = 100):
    """Test step response: 0 -> target RPM."""
    print(f"\n=== STEP RESPONSE TEST (0 -> {target_rpm} RPM) ===")

    tester.enable_pid()
    tester.stop_all()
    time.sleep(0.5)

    print(f"\nSetting target to {target_rpm} RPM...")
    tester.set_all_rpm(target_rpm)
    tester.monitor(duration=5.0, interval=0.25)

    print("\nStopping...")
    tester.stop_all()
    tester.monitor(duration=2.0, interval=0.25)

    tester.disable_pid()


def test_direction_change(tester: PIDTester, rpm: int = 80):
    """Test direction change: forward -> reverse."""
    print(f"\n=== DIRECTION CHANGE TEST (+{rpm} -> -{rpm} RPM) ===")

    tester.enable_pid()

    print(f"\nForward at {rpm} RPM...")
    tester.set_all_rpm(rpm)
    tester.monitor(duration=3.0)

    print(f"\nReverse at -{rpm} RPM...")
    tester.set_all_rpm(-rpm)
    tester.monitor(duration=3.0)

    print("\nStopping...")
    tester.stop_all()
    tester.monitor(duration=2.0)

    tester.disable_pid()


def test_individual_motors(tester: PIDTester, rpm: int = 60):
    """Test each motor individually."""
    print(f"\n=== INDIVIDUAL MOTOR TEST ({rpm} RPM) ===")

    tester.enable_pid()

    for i, name in enumerate(MOTOR_NAMES):
        print(f"\nTesting motor {name}...")
        targets = [0, 0, 0, 0]
        targets[i] = rpm
        tester.set_target_rpm(*targets)
        tester.monitor(duration=2.0)

    tester.stop_all()
    tester.disable_pid()


def interactive_mode(tester: PIDTester):
    """Interactive control mode."""
    print("\n=== INTERACTIVE MODE ===")
    print("Commands:")
    print("  e     - Enable PID")
    print("  d     - Disable PID")
    print("  s     - Stop all motors")
    print("  <num> - Set all motors to <num> RPM (e.g., 100 or -50)")
    print("  m     - Monitor for 5 seconds")
    print("  q     - Quit")

    while True:
        try:
            cmd = input("\n> ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'e':
                tester.enable_pid()
            elif cmd == 'd':
                tester.disable_pid()
            elif cmd == 's':
                tester.stop_all()
            elif cmd == 'm':
                tester.monitor(duration=5.0)
            elif cmd.lstrip('-').isdigit():
                rpm = int(cmd)
                rpm = max(-200, min(200, rpm))
                tester.set_all_rpm(rpm)
            else:
                print("Unknown command")
        except KeyboardInterrupt:
            print("\nInterrupted")
            break

    tester.stop_all()
    tester.disable_pid()


def main():
    print("CLOVER PID Motor Test")
    print("=" * 40)

    tester = PIDTester()

    if not tester.connect():
        sys.exit(1)

    try:
        if len(sys.argv) > 1:
            test = sys.argv[1]
            if test == 'step':
                rpm = int(sys.argv[2]) if len(sys.argv) > 2 else 100
                test_step_response(tester, rpm)
            elif test == 'dir':
                rpm = int(sys.argv[2]) if len(sys.argv) > 2 else 80
                test_direction_change(tester, rpm)
            elif test == 'ind':
                rpm = int(sys.argv[2]) if len(sys.argv) > 2 else 60
                test_individual_motors(tester, rpm)
            elif test == 'int':
                interactive_mode(tester)
            else:
                print(f"Unknown test: {test}")
                print("Available: step, dir, ind, int")
        else:
            # Default: interactive mode
            interactive_mode(tester)

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        tester.stop_all()
        tester.disable_pid()
        tester.disconnect()


if __name__ == '__main__':
    main()
