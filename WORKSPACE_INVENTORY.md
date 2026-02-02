# CLOVER WORKSPACE INVENTORY

**Aggiornato:** 2026-02-02
**Workspace:** `/home/jetsonnano/autonomous-rover`

---

## ARCHITETTURA SISTEMA

```
Jetson Orin Nano (USB Serial)
    ↓ Modbus RTU (115200 baud)
Arduino Uno + Moebius Shield
    ↓ I2C (PCA9685 PWM)
HR8833 H-Bridge × 2
    ↓
JGB 520 Motors × 4 (Mecanum)
    ↓ Hall Encoders
Arduino → Jetson (feedback)
```

---

## PORTE DI RETE UTILIZZATE

| Porta | Servizio | Protocollo | Launcher |
|-------|----------|------------|----------|
| 8081 | RoverService | HTTP + WebSocket | `start_rover_service.py` |
| 8090 | MJPEG Server | HTTP | `src/teleop/mjpeg_server.py` |
| 8080 | WebRTC Teleop | WebRTC | `start_teleop.py` |
| Telegram | Telegram Bot | HTTPS | `start_telegram.py` |

---

## SCRIPT DI AVVIO PRINCIPALI

| Script | Scopo | Comando |
|--------|-------|---------|
| `start_rover_service.py` | Avvia servizio controllo motori | `python3 start_rover_service.py` |
| `start_telegram.py` | Avvia bot Telegram | `python3 start_telegram.py` |
| `start_teleop.py` | Avvia teleoperazione VR | `python3 start_teleop.py` |
| `src/teleop/mjpeg_server.py` | Streaming video web | `python3 src/teleop/mjpeg_server.py --port 8090` |

---

## MODULI PYTHON (src/)

### services/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `rover_service.py` | `RoverService` | HUB centrale - gestisce Arduino, API HTTP/WS, watchdog |

### communication/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `modbus_client.py` | `CloverModbusClient` | Comunicazione Modbus RTU con Arduino |

### control/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `mechanum_drive.py` | `MechanumDrive` | Cinematica Mecanum, conversione velocità→PWM |

### vision/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `camera.py` | `JetsonCamera` | Cattura singola camera GStreamer |
| `stereo_camera.py` | `StereoCamera` | Sistema stereo sincronizzato |
| `detector.py` | `ObjectDetector` | Detection ONNX/TensorRT |
| `stereo_depth.py` | `StereoDepthEstimator` | Stima profondità SGBM |
| `cat_follower.py` | `CatFollower` | Inseguimento gatti autonomo |

### teleop/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `teleop_server.py` | `TeleopServer` | Server VR unificato |
| `streaming_server.py` | `TeleopStreamingServer` | WebRTC H.264 |
| `control_server.py` | `TeleopControlServer` | WebSocket controller |
| `mjpeg_server.py` | `MJPEGServer` | **STREAMING VIDEO WEB** (porta 8090) + **CAT DETECTION** |

#### mjpeg_server.py - Endpoint API
| Endpoint | Metodo | Descrizione |
|----------|--------|-------------|
| `/` | GET | Interfaccia web teleoperazione |
| `/stream` | GET | Stream stereo MJPEG |
| `/stream/left` | GET | Stream camera sinistra |
| `/stream/right` | GET | Stream camera destra |
| `/snapshot` | GET | Singolo frame JPEG |
| `/health` | GET | Stato server |
| `/detection/toggle` | POST | Attiva/disattiva cat detection |
| `/detection/status` | GET | Stato detection e gatti rilevati |

### telegram/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `telegram_bot.py` | `CloverTelegramBot` | Bot Telegram |
| `rover_interface.py` | `RoverInterface` | Client HTTP per RoverService |

### navigation/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `obstacle_avoidance.py` | `ObstacleAvoidance` | Evitamento ostacoli |

### sensors/
| File | Classe Principale | Descrizione |
|------|------------------|-------------|
| `ultrasonic.py` | `UltrasonicSensorArray` | Array ultrasuoni (predisposto) |

---

## SCRIPT DI TEST E CALIBRAZIONE (scripts/)

| Script | Scopo | Note |
|--------|-------|------|
| `calibrate_motors.py` | Calibrazione velocità motori | Open-loop |
| `calibrate_interactive.py` | Calibrazione interattiva PWM | Usa termios |
| `calibrate_pwm_offsets.py` | Offset PWM per velocità uniformi | Misura RPM |
| `test_modbus_motor.py` | Test Modbus raw | DTR/RTS handling |
| `test_motor_speeds.py` | Test risposta motori | Vari PWM |
| `test_pid.py` | Test controllore PID | Con encoder |
| `simple_camera_test.py` | Test base camera | Validazione hardware |
| `test_stereo_detection.py` | Test visione stereo | Detection + depth |

---

## FIRMWARE ARDUINO (firmware/)

### clover_arduino/ (FIRMWARE PRINCIPALE)
| File | Scopo |
|------|-------|
| `clover_arduino.ino` | Main sketch - Modbus slave, motori, encoder |
| `config.h` | Configurazione pin, canali PCA9685, offset calibrazione |
| `motor_driver.h` | Controllo PCA9685 + HR8833 |
| `encoder.h` | Lettura encoder Hall |
| `battery.h` | Monitoraggio batteria |
| `pid_controller.h` | Controllore PID |

### Mappatura Canali Motori (PCA9685)
```
FL (M1): CH2-3 (JP2) - INVERT=true
FR (M2): CH0-1 (JP1) - INVERT=false
RL (M3): CH6-7 (JP4) - INVERT=true
RR (M4): CH4-5 (JP3) - INVERT=false
```

---

## MODELLI ML (models/)

| File | Formato | Uso |
|------|---------|-----|
| `yolo11n.pt` | PyTorch | Detection gatti (COCO class 15) - ATTIVO |
| `yolov8n.pt` | PyTorch | Detection legacy (non usato) |
| `yolov8n.onnx` | ONNX | Detection ottimizzata GPU (legacy) |

---

## CONFIGURAZIONE (config/)

| File | Contenuto |
|------|-----------|
| `rover_config.yaml` | Configurazione generale sistema |
| `telegram_config.yaml` | Token bot, utenti autorizzati |

---

## DIPENDENZE CRITICHE

```
RoverService
    ├── CloverModbusClient → Arduino
    ├── MechanumDrive → Conversione velocità
    └── API HTTP/WS → Client esterni

mjpeg_server.py
    └── cv2.VideoCapture(GStreamer) → Camere CSI

cat_follower.py
    ├── ultralytics YOLO → Detection
    └── stereo_depth.py → Stima profondità
```

---

## SICUREZZA

1. **Watchdog:** Motori stop se nessun comando per 2 secondi
2. **E-Stop:** Registro Modbus 10 per arresto emergenza
3. **Batteria:** Allarme a 10.0V, critico a 9.0V
4. **Moebius Shield:** ATTENZIONE - Supply Power (SX) SOLO batteria!

---

## CAMERA CSI (IMX219)

- **Pipeline GStreamer:** `nvarguscamerasrc sensor-id=X ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! nvvidconv ! videoconvert ! appsink`
- **Sensori:** 0 (sinistra), 1 (destra)
- **Modalità supportate:** 3280x2464@21fps, 1920x1080@30fps, 1280x720@60fps

---

## NOTE OPERATIVE

1. **Una sola app può usare le camere** - Se mjpeg_server.py è attivo, altri script non possono aprire le camere
2. **nvargus-daemon** - Deve essere attivo per le camere CSI: `sudo systemctl restart nvargus-daemon`
3. **Dopo reboot** - Le camere funzionano, errori buffer si risolvono con riavvio
4. **RoverService** - Deve essere attivo per controllo motori via web/telegram

---

## WORKFLOW TIPICO

```bash
# 1. Avviare RoverService (motori)
python3 start_rover_service.py &

# 2. Avviare streaming video
python3 src/teleop/mjpeg_server.py --port 8090 &

# 3. (Opzionale) Avviare Telegram bot
python3 start_telegram.py &

# Accedere via browser:
# http://192.168.1.35:8090/
```

---

*Questo file deve essere aggiornato quando vengono aggiunti/modificati script o servizi.*
