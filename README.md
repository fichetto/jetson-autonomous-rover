# CLOVER - Piattaforma Robotica Mobile Omnidirezionale

Sistema di guida autonoma per rover con ruote Mecanum, basato su Jetson Orin Nano con visione artificiale accelerata da GPU NVIDIA.

## Descrizione

Il progetto CLOVER è una piattaforma robotica mobile omnidirezionale basata su chassis con ruote Mecanum. Il sistema è progettato per movimentazione autonoma in ambienti interni con comunicazione Modbus RTU su interfaccia seriale USB.

## Hardware Setup

### Host - Jetson Orin Nano
- CUDA 12.6
- TensorRT 10.3.0
- DeepStream SDK 7.1
- cuDNN 9.3.0

### Controller - Arduino Uno
- MCU: ATmega328P @ 16 MHz
- Shield: Moebius 4CH Motor Driver
- PWM Controller: PCA9685 (I2C @ 0x40)
- H-Bridge: HR8833 (Dual)
- Comunicazione: Modbus RTU via USB

### Chassis
- **Dimensioni**: 300 x 250 mm
- **Ruote**: 4x Mecanum wheels (movimento omnidirezionale)
- **Motori**: JGB 520 (12V DC, 330 RPM, encoder Hall integrato)
- **Alimentazione**: Batteria LiPo 3S2P (11.1V nominale, 9.0-12.6V range)

### Sensori
- **Camera**: 2x dispositivi video (/dev/video0, /dev/video1)
- **Ultrasuoni**: Predisposto per 6x HC-SR04 (non ancora connessi)
- **Encoder**: 4x Hall effect integrati nei motori

## Architettura Software

```
clover/
├── src/
│   ├── vision/          # Pipeline visione artificiale (CUDA/TensorRT)
│   ├── navigation/      # Algoritmi path planning e obstacle avoidance
│   ├── control/         # Controllo cinematica Mecanum wheels
│   ├── communication/   # Modbus RTU con Arduino
│   └── sensors/         # Gestione sensori (encoder, ultrasuoni)
├── config/              # File configurazione
├── docker/              # Containerizzazione
├── models/              # Modelli AI (ONNX/TensorRT)
├── logs/                # Log operativi
└── tests/               # Unit e integration tests
```

## Architettura Hardware

```
Host PC (Modbus Master) → USB → Arduino Uno (Modbus Slave)
Arduino Uno → I2C → PCA9685 (PWM Generator)
PCA9685 → HR8833 (Motor Driver H-Bridge) → Motori DC
Encoder Motori → Arduino (D2/D3 INT + polling M3/M4)
```

## Pin Mapping Arduino

| Pin | Funzione | Note |
|-----|----------|------|
| D0 (RX) | Serial UART RX | Modbus RTU via USB |
| D1 (TX) | Serial UART TX | Modbus RTU via USB |
| D2 | Encoder M1 | INT0 |
| D3 | Encoder M2 | INT1 |
| A4 (SDA) | I2C Data | PCA9685 |
| A5 (SCL) | I2C Clock | PCA9685 |

**Pin disponibili per espansioni:**
- Digital: D4-D13 (10 pin)
- Analog: A0-A3 (4 pin)

## Features

- [x] Comunicazione Modbus RTU con Arduino
- [x] Controllo cinematica Mecanum wheels
- [x] Feedback encoder 4 motori
- [x] Object detection accelerato GPU
- [x] Path planning in tempo reale
- [x] Obstacle avoidance
- [x] Monitoraggio batteria LiPo
- [ ] Fusione dati sensori ultrasuoni (predisposto)
- [ ] SLAM (mapping ambiente)
- [ ] Deep Learning per semantic segmentation

## Quick Start

```bash
# Installazione dipendenze
pip3 install -r requirements.txt

# Build container Docker
cd docker && docker-compose up -d

# Esecuzione CLOVER
python3 src/main.py
```

## Comunicazione Arduino

- **Protocollo**: Modbus RTU
- **Interfaccia**: USB Serial (/dev/ttyACM0)
- **Baudrate**: 115200
- **Slave ID**: 1

### Registri Modbus

| Registro | Funzione | Tipo |
|----------|----------|------|
| 100-103 | Velocità motori M1-M4 | Holding (R/W) |
| 200-203 | Conteggio encoder M1-M4 | Input (R) |
| 210-213 | Velocità encoder RPM | Input (R) |
| 300 | Emergency Stop | Holding (R/W) |
| 301 | System Mode | Holding (R/W) |
| 302 | Tensione batteria (mV) | Input (R) |

## Specifiche Batteria

| Parametro | Valore |
|-----------|--------|
| Tipo | LiPo 3S2P |
| Tensione nominale | 11.1V |
| Tensione max (carica) | 12.6V |
| Tensione min (cutoff) | 9.0V |
| Warning bassa tensione | 10.0V |

## Note Tecniche

- La shield Moebius non dispone di header passanti. Per accedere ai pin liberi è necessaria una proto-shield stackabile intermedia.
- Gli encoder M3 e M4 richiedono lettura via polling (non interrupt hardware).
- Il sistema supporta fino a 6 sensori ultrasuoni HC-SR04 sui pin digitali disponibili.

## License

MIT License

---
*Tecnocons S.r.l. - Progetto CLOVER v1.0*
