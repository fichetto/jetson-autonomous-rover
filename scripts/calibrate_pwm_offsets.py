#!/usr/bin/env python3
"""
PWM Offset Calibration Script for CLOVER Rover
===============================================
Calibra i motori in open-loop per farli girare alla stessa velocità.

Procedura:
1. Alza il rover (ruote sollevate da terra)
2. Esegui questo script
3. Osserva quale motore gira più veloce/lento
4. Regola gli offset con i tasti indicati
5. Quando soddisfatto, salva i valori

Comandi:
  1-4: Seleziona motore (FL, FR, RL, RR)
  +/- : Aumenta/diminuisci offset del motore selezionato
  a   : Tutti i motori ON al 40%
  s   : STOP tutti
  t   : Test comparativo (uno alla volta)
  p   : Mostra offset correnti
  w   : Salva offset in config.h
  q   : Esci
"""

import subprocess
import serial
import time
import sys
import os
import re

# Configuration
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SLAVE_ID = 1
CONFIG_FILE = '/home/jetsonnano/autonomous-rover/firmware/clover_arduino/config.h'

# Motor names
MOTOR_NAMES = ['FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)']

# Current offsets (will be loaded from config.h)
offsets = [0, 0, 0, 0]
scales = [100, 100, 100, 100]


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
            print(f"Errore connessione: {e}")
            return False

    def close(self):
        if self.serial:
            self.serial.close()

    def write_registers(self, start_addr, values):
        if not self.serial:
            return False
        try:
            count = len(values)
            request = bytes([
                self.slave_id, 0x10,
                (start_addr >> 8) & 0xFF, start_addr & 0xFF,
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

    def set_motors(self, speeds):
        """Set motor speeds (-255 to +255)"""
        values = [s + 255 for s in speeds]
        return self.write_registers(0, values)

    def stop_all(self):
        return self.set_motors([0, 0, 0, 0])


def load_offsets_from_config():
    """Load current PWM offsets from config.h"""
    global offsets, scales
    try:
        with open(CONFIG_FILE, 'r') as f:
            content = f.read()

        # Parse offsets
        for i, name in enumerate(['FL', 'FR', 'RL', 'RR']):
            match = re.search(rf'#define\s+PWM_OFFSET_{name}\s+(-?\d+)', content)
            if match:
                offsets[i] = int(match.group(1))

            match = re.search(rf'#define\s+PWM_SCALE_{name}\s+(\d+)', content)
            if match:
                scales[i] = int(match.group(1))

        return True
    except Exception as e:
        print(f"Errore lettura config: {e}")
        return False


def save_offsets_to_config():
    """Save PWM offsets to config.h"""
    try:
        with open(CONFIG_FILE, 'r') as f:
            content = f.read()

        # Update offsets
        for i, name in enumerate(['FL', 'FR', 'RL', 'RR']):
            content = re.sub(
                rf'(#define\s+PWM_OFFSET_{name}\s+)-?\d+',
                rf'\g<1>{offsets[i]}',
                content
            )

        with open(CONFIG_FILE, 'w') as f:
            f.write(content)

        return True
    except Exception as e:
        print(f"Errore scrittura config: {e}")
        return False


def print_status(selected_motor):
    """Print current offset status"""
    print("\n" + "=" * 50)
    print("OFFSET CORRENTI:")
    print("=" * 50)
    for i, name in enumerate(MOTOR_NAMES):
        marker = " >>>" if i == selected_motor else "    "
        print(f"{marker} [{i+1}] {name}: offset={offsets[i]:+3d}, scale={scales[i]}%")
    print("=" * 50)
    print("Positivo = più veloce, Negativo = più lento")
    print()


def print_help():
    print("""
Comandi:
  1-4  : Seleziona motore
  +/-  : Aumenta/diminuisci offset (+5/-5)
  a    : Tutti i motori ON al 40%
  s    : STOP tutti
  t    : Test comparativo (uno alla volta)
  p    : Mostra offset
  w    : Salva in config.h (richiede ricompilazione!)
  q    : Esci
""")


def main():
    print("=" * 60)
    print("    CALIBRAZIONE PWM OFFSET - CLOVER ROVER")
    print("=" * 60)
    print()
    print("IMPORTANTE: Alza il rover! Le ruote devono essere sollevate!")
    print()

    # Load current offsets
    if load_offsets_from_config():
        print("Offset caricati da config.h")
    else:
        print("Usando offset di default (0)")

    input("\nPremi INVIO quando il rover è sollevato...")

    # Connect
    print("\nConnessione...")
    client = ModbusClient(PORT, BAUDRATE, SLAVE_ID)
    if not client.connect():
        print("Impossibile connettersi!")
        sys.exit(1)
    print("Connesso!")

    selected_motor = 0
    motors_running = False
    base_speed = 100  # ~40%

    print_status(selected_motor)
    print_help()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if cmd == 'q':
                break

            elif cmd in ['1', '2', '3', '4']:
                selected_motor = int(cmd) - 1
                print(f"Selezionato: {MOTOR_NAMES[selected_motor]}")

            elif cmd == '+':
                offsets[selected_motor] = min(50, offsets[selected_motor] + 5)
                print(f"{MOTOR_NAMES[selected_motor]}: offset = {offsets[selected_motor]:+d}")
                print("NOTA: Ricompila il firmware per applicare!")

            elif cmd == '-':
                offsets[selected_motor] = max(-50, offsets[selected_motor] - 5)
                print(f"{MOTOR_NAMES[selected_motor]}: offset = {offsets[selected_motor]:+d}")
                print("NOTA: Ricompila il firmware per applicare!")

            elif cmd == 'a':
                print(f"Tutti i motori al {base_speed/255*100:.0f}%...")
                client.set_motors([base_speed, base_speed, base_speed, base_speed])
                motors_running = True
                print("Osserva le ruote: quale gira più veloce/lenta?")

            elif cmd == 's':
                client.stop_all()
                motors_running = False
                print("STOP")

            elif cmd == 't':
                print("\nTest comparativo - ogni motore per 2 secondi:")
                for i in range(4):
                    print(f"  {MOTOR_NAMES[i]}...")
                    speeds = [0, 0, 0, 0]
                    speeds[i] = base_speed
                    client.set_motors(speeds)
                    time.sleep(2)
                client.stop_all()
                print("Test completato")

            elif cmd == 'p':
                print_status(selected_motor)

            elif cmd == 'w':
                if save_offsets_to_config():
                    print("\n" + "=" * 50)
                    print("OFFSET SALVATI in config.h!")
                    print("=" * 50)
                    print("Per applicare le modifiche:")
                    print("  1. Ricompila il firmware:")
                    print("     cd firmware/clover_arduino")
                    print("     arduino-cli compile --fqbn arduino:avr:uno .")
                    print("  2. Carica il firmware:")
                    print("     arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno .")
                    print("=" * 50)
                else:
                    print("Errore nel salvataggio!")

            elif cmd == 'h' or cmd == '?':
                print_help()

            elif cmd == '':
                pass  # Ignore empty input

            else:
                print("Comando sconosciuto. Premi 'h' per aiuto.")

    except KeyboardInterrupt:
        print("\n\nInterrotto!")

    finally:
        client.stop_all()
        client.close()
        print("\nMotori fermati. Connessione chiusa.")


if __name__ == "__main__":
    main()
