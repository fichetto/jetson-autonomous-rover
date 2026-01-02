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
│   ├── services/        # RoverService - servizio unificato controllo rover
│   ├── telegram/        # Bot Telegram per controllo remoto
│   ├── teleop/          # Server video MJPEG e interfaccia web
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

### Architettura Servizi Unificata

Il sistema utilizza un'architettura a servizi che permette a più client di controllare il rover senza conflitti sulla connessione seriale:

```
                    ┌─────────────────────────────────────┐
                    │         RoverService (8082)         │
                    │   Gestione centralizzata Arduino    │
                    │   - Connessione Modbus unica        │
                    │   - API HTTP/WebSocket              │
                    │   - Arbitraggio priorità            │
                    │   - Watchdog sicurezza              │
                    └──────────────┬──────────────────────┘
                                   │ USB Serial
                                   ▼
                    ┌─────────────────────────────────────┐
                    │        Arduino (Modbus Slave)       │
                    └─────────────────────────────────────┘
                                   ▲
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
┌───────┴───────┐        ┌─────────┴────────┐       ┌─────────┴────────┐
│  Interfaccia  │        │   Telegram Bot   │       │  Guida Autonoma  │
│  Web (8090)   │        │                  │       │   (futuro)       │
│               │        │  /start /status  │       │                  │
│  MJPEG Stream │        │  /manual /auto   │       │  Path Planning   │
│  Controlli    │        │  /stop /estop    │       │  Obstacle Avoid  │
└───────────────┘        └──────────────────┘       └──────────────────┘
```

**Porte di servizio:**
| Porta | Servizio | Descrizione |
|-------|----------|-------------|
| 8082  | RoverService | API controllo motori (HTTP + WebSocket) |
| 8090  | MJPEG Server | Streaming video stereo + interfaccia web |

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
- [x] **VR Teleoperation** con Meta Quest 3
- [x] **Stereo Camera Streaming** (MJPEG)
- [x] **Interfaccia Web** per controllo manuale
- [x] **Bot Telegram** per controllo remoto e monitoraggio
- [x] **Architettura unificata** multi-client
- [ ] Fusione dati sensori ultrasuoni (predisposto)
- [ ] SLAM (mapping ambiente)
- [ ] Deep Learning per semantic segmentation

## Quick Start

```bash
# Installazione dipendenze
pip3 install -r requirements.txt

# Avvio servizio unificato (RoverService)
python3 start_rover_service.py --serial /dev/ttyUSB0

# Avvio server video MJPEG (in altro terminale)
python3 src/teleop/mjpeg_server.py --port 8090

# Avvio bot Telegram (in altro terminale)
python3 start_telegram.py --service-url http://localhost:8082
```

### Accesso Interfacce

- **Interfaccia Web**: http://IP_JETSON:8090
- **Telegram Bot**: Cerca il bot configurato e usa /start

## VR Teleoperation (Meta Quest 3)

Controlla CLOVER in prima persona con visione stereo e joystick VR.

### Avvio Server

```bash
# Avvio con configurazione automatica
./start_teleop.py

# Oppure modalità test (senza Arduino)
./start_teleop.py --no-modbus

# Risoluzione ridotta per WiFi lento
./start_teleop.py --width 640 --height 480
```

### Architettura Teleop

```
┌─────────────────┐     WiFi      ┌─────────────────┐
│   Meta Quest 3  │◄────────────► │  Jetson Orin    │
│                 │               │                 │
│  ┌───────────┐  │    MJPEG     │  ┌───────────┐  │
│  │Video (VR) │◄─┼──────────────┼──┤Stereo Cam │  │
│  └───────────┘  │  :8090       │  │ (IMX219x2)│  │
│                 │               │  └───────────┘  │
│  ┌───────────┐  │  WebSocket   │  ┌───────────┐  │
│  │Controllers├──┼──────────────┼─►│RoverSvc   │  │
│  └───────────┘  │  :8082       │  │ (Modbus)  │  │
└─────────────────┘               └─────────────────┘
```

### Controlli Quest 3

| Input | Azione |
|-------|--------|
| Stick Sinistro Y | Avanti / Indietro |
| Stick Sinistro X | Strafe Sinistra / Destra |
| Stick Destro X | Rotazione |
| Trigger Sinistro | Modalità Lenta (precisione) |
| Trigger Destro | Modalità Veloce |
| Pulsante B | **EMERGENCY STOP** |
| Pulsante Y | Toggle UI |

### Setup App Unity

Vedi [unity/CloverVR/README.md](unity/CloverVR/README.md) per istruzioni complete su come configurare il progetto Unity per Quest 3.

## Bot Telegram

Il bot Telegram permette di controllare e monitorare CLOVER da remoto.

### Configurazione

1. Crea un bot con [@BotFather](https://t.me/BotFather) su Telegram
2. Copia il token in `config/telegram_config.yaml`
3. Avvia il bot: `python3 start_telegram.py`

### Comandi Disponibili

| Comando | Descrizione |
|---------|-------------|
| `/start` | Avvia il bot e mostra menu |
| `/status` | Stato rover (batteria, modo, connessione) |
| `/connect` | Connetti ad Arduino |
| `/disconnect` | Disconnetti Arduino |
| `/manual` | Modalità controllo manuale |
| `/auto` | Modalità guida autonoma |
| `/stop` | Ferma i motori |
| `/estop` | **EMERGENCY STOP** |
| `/help` | Aiuto comandi |

### Controllo Movimento (Modalità Manuale)

In modalità manuale, usa i pulsanti inline per controllare il rover:
- Frecce direzionali per movimento
- Pulsanti rotazione per girare sul posto
- Slider velocità per regolare la potenza

## RoverService API

Il RoverService espone API HTTP e WebSocket per il controllo del rover.

### Endpoint HTTP

| Metodo | Endpoint | Descrizione |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/status` | Stato completo rover |
| POST | `/connect` | Connetti Arduino `{"port": "/dev/ttyUSB0"}` |
| POST | `/disconnect` | Disconnetti Arduino |
| POST | `/move` | Movimento `{"vx": 0.5, "vy": 0, "wz": 0}` |
| POST | `/stop` | Ferma motori |
| POST | `/estop` | Emergency stop |
| POST | `/mode` | Cambia modo `{"mode": "manual"}` |

### WebSocket `/ws`

Connessione WebSocket per controllo real-time:

```json
// Imposta modalità
{"type": "set_mode", "mode": "teleop"}

// Comando movimento (stile controller)
{
  "type": "controller_state",
  "left_thumbstick_x": 0.0,
  "left_thumbstick_y": 0.5,
  "right_thumbstick_x": 0.0
}
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
