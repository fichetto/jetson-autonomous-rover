# Autonomous Rover - Jetson Orin Nano

Sistema di guida autonoma per rover con visione artificiale accelerata da GPU NVIDIA.

## Hardware Setup

### Jetson Orin Nano
- CUDA 12.6
- TensorRT 10.3.0
- DeepStream SDK 7.1
- cuDNN 9.3.0

### Rover Specifications
- **Ruote**: Mechanum wheels (movimento omnidirezionale)
- **Sensori**: 6 sensori ultrasuoni
- **Comunicazione**: Modbus RTU via USB-Serial
- **Controller**: Arduino
- **Camera**: 2x dispositivi video disponibili (/dev/video0, /dev/video1)

## Architettura Software

```
autonomous-rover/
├── src/
│   ├── vision/          # Pipeline visione artificiale (CUDA/TensorRT)
│   ├── navigation/      # Algoritmi path planning e obstacle avoidance
│   ├── control/         # Controllo motori Mechanum
│   ├── communication/   # Modbus RTU con Arduino
│   ├── sensors/         # Gestione sensori ultrasuoni
│   └── utils/           # Utility e helper functions
├── config/              # File configurazione
├── docker/              # Containerizzazione
├── models/              # Modelli AI (ONNX/TensorRT)
├── logs/                # Log operativi
└── tests/               # Unit e integration tests
```

## Features

- [x] Comunicazione Modbus RTU con Arduino
- [x] Controllo cinematica Mechanum wheels
- [x] Fusione dati 6 sensori ultrasuoni
- [x] Object detection accelerato GPU
- [x] Path planning in tempo reale
- [x] Obstacle avoidance con sensor fusion
- [ ] SLAM (mapping ambiente)
- [ ] Deep Learning per semantic segmentation

## Quick Start

```bash
# Installazione dipendenze
pip3 install -r requirements.txt

# Build container Docker
cd docker && docker-compose up -d

# Esecuzione rover
python3 src/main.py
```

## Comunicazione Arduino

- **Protocollo**: Modbus RTU
- **Interfaccia**: USB Serial (/dev/ttyACM* o /dev/ttyUSB*)
- **Baudrate**: 115200 (configurabile)
- **Registri Modbus**:
  - Lettura sensori ultrasuoni
  - Controllo velocità motori
  - Stato sistema

## License

MIT License
