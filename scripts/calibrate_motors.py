#!/usr/bin/env python3
"""
Motor Calibration Script for CLOVER Rover
==========================================
Calibra i motori in open-loop per farli girare alla stessa velocità
senza usare il feedback encoder.

Procedura:
1. Alza il rover (ruote sollevate da terra)
2. Esegui questo script
3. Osserva quale motore gira più veloce/lento
4. Regola i valori PWM_OFFSET_xx in config.h
"""

import sys
import time
from pymodbus.client import ModbusSerialClient

# Modbus registers
HREG_MOTOR_FL_SPEED = 0
HREG_MOTOR_FR_SPEED = 1
HREG_MOTOR_RL_SPEED = 2
HREG_MOTOR_RR_SPEED = 3
HREG_EMERGENCY_STOP = 10
IREG_BATTERY_VOLTAGE = 20

# Speed mapping: 255 = stop, 0-254 = reverse, 256-510 = forward
MOTOR_STOP = 255

def speed_to_register(speed):
    """Convert -255..+255 speed to 0-510 register value"""
    return speed + 255

def connect():
    """Connect to Arduino via Modbus RTU"""
    client = ModbusSerialClient(
        port='/dev/ttyUSB0',
        baudrate=115200,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=1
    )
    if not client.connect():
        print("ERRORE: Impossibile connettersi ad Arduino")
        sys.exit(1)
    return client

def stop_all(client):
    """Stop all motors"""
    client.write_registers(HREG_MOTOR_FL_SPEED, [MOTOR_STOP, MOTOR_STOP, MOTOR_STOP, MOTOR_STOP], slave=1)

def set_motor_speeds(client, fl, fr, rl, rr):
    """Set individual motor speeds (-255 to +255)"""
    values = [
        speed_to_register(fl),
        speed_to_register(fr),
        speed_to_register(rl),
        speed_to_register(rr)
    ]
    client.write_registers(HREG_MOTOR_FL_SPEED, values, slave=1)

def read_battery(client):
    """Read battery voltage"""
    result = client.read_input_registers(IREG_BATTERY_VOLTAGE, count=1, slave=1)
    if result.isError():
        return 0
    return result.registers[0] / 1000.0

def test_single_motor(client, motor_name, motor_index, speed, duration=3):
    """Test a single motor"""
    speeds = [0, 0, 0, 0]
    speeds[motor_index] = speed

    print(f"\n  Testing {motor_name} at speed {speed}...")
    set_motor_speeds(client, *speeds)
    time.sleep(duration)
    stop_all(client)
    print(f"  {motor_name} stopped")

def test_all_motors_same_speed(client, speed, duration=5):
    """Test all motors at the same speed"""
    print(f"\n  All motors at speed {speed}...")
    print("  OSSERVA: quale ruota gira più veloce/lenta?")
    set_motor_speeds(client, speed, speed, speed, speed)
    time.sleep(duration)
    stop_all(client)

def main():
    print("=" * 60)
    print("     CALIBRAZIONE MOTORI CLOVER (Open-Loop)")
    print("=" * 60)
    print()
    print("IMPORTANTE: Alza il rover! Le ruote devono essere sollevate!")
    print()

    input("Premi INVIO quando il rover è sollevato...")

    client = connect()
    print("Connesso ad Arduino")

    # Check battery
    voltage = read_battery(client)
    print(f"Tensione batteria: {voltage:.2f}V")
    if voltage < 9.0:
        print("ATTENZIONE: Batteria scarica!")

    # Disable emergency stop
    client.write_register(HREG_EMERGENCY_STOP, 0, slave=1)

    try:
        while True:
            print("\n" + "=" * 60)
            print("MENU CALIBRAZIONE")
            print("=" * 60)
            print("1. Test singolo motore")
            print("2. Test tutti i motori (stessa velocità)")
            print("3. Test velocità variabili")
            print("4. Test direzione (avanti/indietro)")
            print("5. Mostra valori calibrazione suggeriti")
            print("0. Esci")
            print()

            choice = input("Scelta: ").strip()

            if choice == '0':
                break

            elif choice == '1':
                print("\nMotori: 0=FL, 1=FR, 2=RL, 3=RR")
                motor = int(input("Numero motore (0-3): "))
                speed = int(input("Velocità (1-255): "))
                motor_names = ['FL (Front Left)', 'FR (Front Right)',
                              'RL (Rear Left)', 'RR (Rear Right)']
                test_single_motor(client, motor_names[motor], motor, speed)

            elif choice == '2':
                speed = int(input("Velocità per tutti (1-255): "))
                test_all_motors_same_speed(client, speed)

            elif choice == '3':
                print("\nTest velocità crescenti...")
                for speed in [50, 100, 150, 200]:
                    print(f"\n--- Velocità {speed} ---")
                    set_motor_speeds(client, speed, speed, speed, speed)
                    time.sleep(2)
                stop_all(client)

            elif choice == '4':
                print("\nTest AVANTI (5 sec)...")
                set_motor_speeds(client, 100, 100, 100, 100)
                time.sleep(5)
                stop_all(client)
                time.sleep(1)
                print("Test INDIETRO (5 sec)...")
                set_motor_speeds(client, -100, -100, -100, -100)
                time.sleep(5)
                stop_all(client)

            elif choice == '5':
                print("\n" + "=" * 60)
                print("COME CALIBRARE I MOTORI")
                print("=" * 60)
                print("""
1. Usa l'opzione 2 per far girare tutti i motori a velocità 150

2. Osserva le ruote:
   - Se un motore gira PIÙ VELOCE: usa offset NEGATIVO
   - Se un motore gira PIÙ LENTO: usa offset POSITIVO

3. Modifica config.h:

   // Esempio: FR è più veloce, RL è più lento
   #define PWM_OFFSET_FL   0
   #define PWM_OFFSET_FR  -10   // rallenta FR
   #define PWM_OFFSET_RL  +15   // accelera RL
   #define PWM_OFFSET_RR   0

4. Ricompila e ricarica il firmware

5. Ripeti il test fino a quando tutte le ruote girano
   alla stessa velocità
""")

    except KeyboardInterrupt:
        print("\n\nInterrotto!")
    finally:
        stop_all(client)
        client.close()
        print("\nMotori fermati. Connessione chiusa.")

if __name__ == "__main__":
    main()
