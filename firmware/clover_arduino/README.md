# CLOVER Arduino Firmware

Firmware per Arduino Uno che gestisce il controllo motori e la comunicazione Modbus RTU con Jetson Orin Nano.

## Hardware Supportato

- **Microcontroller**: Arduino Uno (ATmega328P @ 16MHz)
- **Motor Driver Shield**: Moebius 4CH
  - PCA9685 PWM Controller (I2C @ 0x40)
  - HR8833 Dual H-Bridge (x2)
- **Motori**: JGB 520 (12V DC, 330 RPM, encoder Hall)
- **Batteria**: LiPo 3S2P (11.1V nominale)

## Alimentazione Moebius Shield

La Moebius Shield ha **DUE morsettiere di alimentazione separate**:

```
┌─────────────────────────────────────────────────────────────────┐
│                      MOEBIUS SHIELD                             │
│                                                                 │
│  [Supply Power]              [Servo Power]                      │
│   + | -                       + | -                             │
│   │   │                       │   │                             │
│   │   └── GND Batteria        │   └── GND (se servo separati)   │
│   └────── +11.1V Batteria     └────── +5-8.4V (se servo usati)  │
│                                                                 │
│            MOTORI                      SERVO                    │
└─────────────────────────────────────────────────────────────────┘
```

### Morsettiera Supply Power (SINISTRA) - Per MOTORI
| Pin | Collegamento |
|-----|--------------|
| + | Positivo batteria LiPo 3S (11.1V) |
| - | Negativo batteria (GND) |

### Morsettiera Servo Power (DESTRA) - Per SERVO (opzionale)
| Pin | Collegamento |
|-----|--------------|
| + | Alimentazione servo 5-8.4V (se usati) |
| - | GND |

### Jumper 5V - VCC50 - VM (Alimentazione Servo)

Sulla scheda è presente un jumper a 3 pin per selezionare l'alimentazione dei servo:

```
    5V    VCC50    VM
    [ ]────[ ]────[ ]
         ▲
    Jumper qui
```

| Posizione Jumper | Effetto |
|------------------|---------|
| **5V - VCC50** | Servo alimentati da 5V Arduino (CONSIGLIATO) |
| **VCC50 - VM** | Servo alimentati da VM (tensione esterna) |

### ATTENZIONE - Errore da evitare

**NON collegare MAI la batteria 11.1V alla morsettiera "Servo Power"!**

Se il jumper è su VCC50-VM e si collega alta tensione a "Servo Power":
```
Servo Power 11.1V → VM → jumper → VCC50 → 5V Arduino → USB Host = DANNO!
```

Questo errore può bruciare l'Arduino e le porte USB del dispositivo host (Jetson).

## Pin Mapping

| Pin | Funzione | Descrizione |
|-----|----------|-------------|
| D0 (RX) | Modbus RTU | Serial RX |
| D1 (TX) | Modbus RTU | Serial TX |
| D2 | Encoder M1 | INT0 (interrupt) |
| D3 | Encoder M2 | INT1 (interrupt) |
| D4 | Encoder M3 | Polling |
| D5 | Encoder M4 | Polling |
| A0 | Battery | Voltage divider input |
| A4 (SDA) | I2C | PCA9685 Data |
| A5 (SCL) | I2C | PCA9685 Clock |

## PCA9685 Channel Mapping

| Canale | Funzione | Motore |
|--------|----------|--------|
| CH0 | IN1 | Front Left (M1) |
| CH1 | IN2 | Front Left (M1) |
| CH2 | IN1 | Front Right (M2) |
| CH3 | IN2 | Front Right (M2) |
| CH4 | IN1 | Rear Left (M3) |
| CH5 | IN2 | Rear Left (M3) |
| CH6 | IN1 | Rear Right (M4) |
| CH7 | IN2 | Rear Right (M4) |

## Dipendenze (Librerie Arduino)

Installa le seguenti librerie tramite Arduino Library Manager:

1. **Adafruit PWM Servo Driver Library**
   - Per il controllo del PCA9685
   - `Sketch > Include Library > Manage Libraries > "Adafruit PWM Servo"`

2. **ModbusRtu**
   - Per la comunicazione Modbus RTU slave
   - `Sketch > Include Library > Manage Libraries > "ModbusRtu"`
   - Oppure: https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino

3. **Wire** (inclusa in Arduino IDE)
   - Per comunicazione I2C

## Installazione

1. Apri Arduino IDE
2. Installa le librerie richieste (vedi sopra)
3. Apri `clover_arduino.ino`
4. Seleziona Board: `Arduino Uno`
5. Seleziona la porta seriale corretta
6. Carica il firmware

## Modbus Register Map

### Holding Registers (Read/Write)

| Registro | Funzione | Range |
|----------|----------|-------|
| 100 | Motor FL Speed | 0-510 (255=stop) |
| 101 | Motor FR Speed | 0-510 (255=stop) |
| 102 | Motor RL Speed | 0-510 (255=stop) |
| 103 | Motor RR Speed | 0-510 (255=stop) |
| 300 | Emergency Stop | 0=off, 1=on |
| 301 | System Mode | 0=manual, 1=auto |

### Input Registers (Read Only)

| Registro | Funzione | Unità |
|----------|----------|-------|
| 200-203 | Encoder Counts M1-M4 | ticks |
| 210-213 | Encoder Speed M1-M4 | RPM |
| 302 | Battery Voltage | mV |

### Speed Value Mapping

```
Registro → Velocità:
  0-254   → -255 to -1 (reverse)
  255     → 0 (stop)
  256-510 → +1 to +255 (forward)

Formula: speed = register_value - 255
```

## Comunicazione

- **Protocollo**: Modbus RTU
- **Baudrate**: 115200
- **Slave ID**: 1
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1

## Safety Features

1. **Watchdog Timer**: Motori si fermano se non ricevono comandi per 2 secondi
2. **Battery Protection**: Emergency stop automatico se batteria < 9.0V
3. **Emergency Stop**: Registro 300 per stop immediato

## Circuito Voltage Divider (Batteria)

Per monitorare la batteria LiPo 3S (max 12.6V), serve un voltage divider:

```
Battery+ ----[10K]----+----[3.3K]---- GND
                      |
                      +-------------- A0
```

Ratio = (10K + 3.3K) / 3.3K = 4.03

## Test

Per testare il firmware senza Jetson, usa un terminale Modbus come:
- **QModMaster** (Windows)
- **mbpoll** (Linux)

Esempio con mbpoll:
```bash
# Leggi tensione batteria
mbpoll -a 1 -b 115200 -t 4 -r 302 -c 1 /dev/ttyACM0

# Scrivi velocità motore FL (forward 50%)
mbpoll -a 1 -b 115200 -t 4 -r 100 /dev/ttyACM0 383
```

## Troubleshooting

### Motori non girano
1. Verifica alimentazione PCA9685 (VCC e V+)
2. Controlla indirizzo I2C con scanner
3. Verifica connessioni HR8833

### Encoder non funzionano
1. Verifica connessioni 5V/GND encoder
2. Controlla pin di segnale
3. Per M3/M4: assicurati che il polling sia attivo

### Comunicazione Modbus fallisce
1. Verifica baudrate (115200)
2. Controlla connessione USB
3. Verifica che Serial Monitor sia chiuso

## File Structure

```
firmware/clover_arduino/
├── clover_arduino.ino   # Main sketch
├── config.h             # Configuration & pin mapping
├── motor_driver.h       # PCA9685 + HR8833 control
├── encoder.h            # Encoder reading
├── battery.h            # Battery monitoring
└── README.md            # This file
```

## Licenza

MIT License - Tecnocons S.r.l.
