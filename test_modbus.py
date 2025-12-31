#!/usr/bin/env python3
"""
CLOVER - Test comunicazione Modbus RTU con Arduino
Test semplice senza dipendenze esterne
"""

import struct
import time

# Prova a importare serial
try:
    import serial
except ImportError:
    print("ERRORE: pyserial non installato")
    print("Esegui: sudo apt install python3-serial")
    exit(1)

# Configurazione
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
SLAVE_ID = 1

def crc16_modbus(data):
    """Calcola CRC16 Modbus"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_read_input_registers(slave_id, start_addr, count):
    """Costruisce frame Modbus per lettura input registers (function 0x04)"""
    frame = struct.pack('>BBHH', slave_id, 0x04, start_addr, count)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)
    return frame

def build_read_holding_registers(slave_id, start_addr, count):
    """Costruisce frame Modbus per lettura holding registers (function 0x03)"""
    frame = struct.pack('>BBHH', slave_id, 0x03, start_addr, count)
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc)
    return frame

def parse_response(response, expected_bytes):
    """Parsea risposta Modbus"""
    if len(response) < 5:
        return None, "Risposta troppo corta"

    slave_id = response[0]
    function = response[1]

    # Check for exception
    if function & 0x80:
        return None, f"Exception: {response[2]}"

    byte_count = response[2]
    if len(response) < 3 + byte_count + 2:
        return None, "Risposta incompleta"

    # Estrai dati
    data = response[3:3+byte_count]

    # Converti in registri (16-bit big endian)
    registers = []
    for i in range(0, len(data), 2):
        if i + 1 < len(data):
            registers.append(struct.unpack('>H', data[i:i+2])[0])

    return registers, None

def main():
    print("=" * 60)
    print("       CLOVER - Test Comunicazione Modbus RTU")
    print("=" * 60)
    print(f"\nPorta: {SERIAL_PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Slave ID: {SLAVE_ID}")
    print()

    # Apri seriale
    print("[1] Apertura porta seriale...")
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        print("    OK - Porta aperta")
    except Exception as e:
        print(f"    ERRORE: {e}")
        return 1

    # Attesa stabilizzazione Arduino (dopo reset)
    time.sleep(2)
    ser.reset_input_buffer()

    tests_passed = 0
    errors = 0

    # Test 1: Lettura tensione batteria (Input Register 20)
    print("\n[2] Lettura tensione batteria (Input Register 20)...")
    try:
        frame = build_read_input_registers(SLAVE_ID, 20, 1)
        ser.write(frame)
        time.sleep(0.1)
        response = ser.read(32)

        if len(response) > 0:
            registers, error = parse_response(response, 1)
            if error:
                print(f"    ERRORE: {error}")
                errors += 1
            else:
                voltage_mv = registers[0]
                voltage_v = voltage_mv / 1000.0
                print(f"    OK - Tensione: {voltage_mv} mV ({voltage_v:.2f} V)")
                tests_passed += 1
        else:
            print("    ERRORE: Nessuna risposta")
            errors += 1
    except Exception as e:
        print(f"    ERRORE: {e}")
        errors += 1

    time.sleep(0.1)

    # Test 2: Lettura encoder counts (Input Registers 0-3)
    print("\n[3] Lettura encoder counts (Input Registers 0-3)...")
    try:
        frame = build_read_input_registers(SLAVE_ID, 0, 4)
        ser.write(frame)
        time.sleep(0.1)
        response = ser.read(32)

        if len(response) > 0:
            registers, error = parse_response(response, 4)
            if error:
                print(f"    ERRORE: {error}")
                errors += 1
            else:
                print(f"    OK - Encoder counts: M1={registers[0]}, M2={registers[1]}, "
                      f"M3={registers[2]}, M4={registers[3]}")
                tests_passed += 1
        else:
            print("    ERRORE: Nessuna risposta")
            errors += 1
    except Exception as e:
        print(f"    ERRORE: {e}")
        errors += 1

    time.sleep(0.1)

    # Test 3: Lettura velocità motori (Holding Registers 0-3)
    print("\n[4] Lettura velocità motori (Holding Registers 0-3)...")
    try:
        frame = build_read_holding_registers(SLAVE_ID, 0, 4)
        ser.write(frame)
        time.sleep(0.1)
        response = ser.read(32)

        if len(response) > 0:
            registers, error = parse_response(response, 4)
            if error:
                print(f"    ERRORE: {error}")
                errors += 1
            else:
                # 255 = stop, <255 = reverse, >255 = forward
                print(f"    OK - Registri velocità: FL={registers[0]}, FR={registers[1]}, "
                      f"RL={registers[2]}, RR={registers[3]}")
                if all(r == 255 for r in registers):
                    print("    (Tutti i motori fermi - corretto)")
                tests_passed += 1
        else:
            print("    ERRORE: Nessuna risposta")
            errors += 1
    except Exception as e:
        print(f"    ERRORE: {e}")
        errors += 1

    time.sleep(0.1)

    # Test 4: Lettura RPM encoder (Input Registers 10-13)
    print("\n[5] Lettura RPM encoder (Input Registers 10-13)...")
    try:
        frame = build_read_input_registers(SLAVE_ID, 10, 4)
        ser.write(frame)
        time.sleep(0.1)
        response = ser.read(32)

        if len(response) > 0:
            registers, error = parse_response(response, 4)
            if error:
                print(f"    ERRORE: {error}")
                errors += 1
            else:
                # Offset 32768 per valori signed
                rpms = [r - 32768 for r in registers]
                print(f"    OK - RPM: M1={rpms[0]}, M2={rpms[1]}, M3={rpms[2]}, M4={rpms[3]}")
                tests_passed += 1
        else:
            print("    ERRORE: Nessuna risposta")
            errors += 1
    except Exception as e:
        print(f"    ERRORE: {e}")
        errors += 1

    # Chiudi seriale
    ser.close()

    # Risultati
    print("\n" + "=" * 60)
    print("                    RISULTATI TEST")
    print("=" * 60)
    print(f"  Test superati: {tests_passed}/4")
    print(f"  Errori: {errors}")

    if errors == 0:
        print("\n  COMUNICAZIONE MODBUS: OK")
        print("  Arduino CLOVER funzionante!")
    else:
        print("\n  COMUNICAZIONE MODBUS: PROBLEMI RILEVATI")
        print("  Verificare connessioni e firmware.")

    print("=" * 60)

    return 0 if errors == 0 else 1


if __name__ == "__main__":
    exit(main())
