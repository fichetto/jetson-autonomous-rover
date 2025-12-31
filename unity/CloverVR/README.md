# CLOVER VR - Meta Quest 3 Teleoperation App

Applicazione Unity per il controllo remoto del rover CLOVER tramite Meta Quest 3 con visione stereo.

## Requisiti

### Software
- Unity 2022.3 LTS o superiore
- Meta XR SDK (Oculus Integration)
- Unity WebRTC Package
- NativeWebSocket Package

### Hardware
- Meta Quest 3
- Rete WiFi condivisa con il rover CLOVER

## Setup Progetto Unity

### 1. Creare Nuovo Progetto
```
File → New Project → 3D (URP) → "CloverVR"
```

### 2. Installare Packages

**Via Package Manager:**
```
Window → Package Manager → + → Add package from git URL
```

Aggiungere:
- `com.unity.webrtc` - WebRTC per video streaming
- `com.meta.xr.sdk.all` - Meta XR SDK completo

**Via Unity Asset Store:**
- Oculus Integration (se non incluso in Meta XR SDK)

**Manualmente:**
- NativeWebSocket: https://github.com/endel/NativeWebSocket
  - Scaricare e copiare in `Assets/Plugins/`

### 3. Configurare Build Settings
```
File → Build Settings
- Platform: Android
- Texture Compression: ASTC
```

### 4. Configurare Player Settings
```
Edit → Project Settings → Player
```

**Android Tab:**
- Company Name: "YourCompany"
- Product Name: "CloverVR"
- Package Name: "com.yourcompany.clovervr"

**XR Plug-in Management:**
- Abilita "Oculus" per Android

**Oculus (sotto XR Plug-in Management):**
- Target Devices: Quest 3
- Stereo Rendering Mode: Multiview

### 5. Configurare Scena

1. Rimuovere Main Camera predefinita
2. Aggiungere:
   - `XR Origin (Action-based)` (dal menu Create → XR)
   - O usare `OVRCameraRig` (da Oculus Integration)

3. Creare GameObject "CloverTeleop" con:
   - `CloverTeleopMain.cs`
   - `WebRTCManager.cs`
   - `ControllerManager.cs`
   - `StereoRenderer.cs`

4. Configurare riferimenti nei componenti

## Architettura

```
┌─────────────────────────────────────────────────────┐
│                   Quest 3                           │
│  ┌──────────────┐  ┌──────────────┐                │
│  │ WebRTCManager│  │ Controller   │                │
│  │ (video RX)   │  │ Manager (TX) │                │
│  └──────┬───────┘  └──────┬───────┘                │
│         │                 │                         │
│         ▼                 ▼                         │
│  ┌──────────────┐  ┌──────────────┐                │
│  │ StereoRenderer│  │ Haptic      │                │
│  │ (L/R eyes)   │  │ Feedback    │                │
│  └──────────────┘  └──────────────┘                │
└─────────────────────────────────────────────────────┘
                    │ WiFi │
                    ▼      ▼
┌─────────────────────────────────────────────────────┐
│               CLOVER Jetson Orin Nano               │
│  Port 8080: WebRTC Video (stereo SBS)              │
│  Port 8081: WebSocket Control                       │
└─────────────────────────────────────────────────────┘
```

## Controlli

| Controller | Input | Azione |
|------------|-------|--------|
| Left Stick | Y | Avanti/Indietro |
| Left Stick | X | Strafe Sinistra/Destra |
| Right Stick | X | Rotazione |
| Left Trigger | Hold | Modalità lenta (precisione) |
| Right Trigger | Hold | Modalità veloce |
| Button B | Press | EMERGENCY STOP |
| Button Y | Press | Toggle UI |
| Both Sticks | Click | Ricenter visuale |

## Configurazione IP

L'IP del rover può essere configurato:
1. Nel Inspector di `CloverTeleopMain`
2. Runtime tramite menu settings
3. Automaticamente salvato in PlayerPrefs

## Build & Deploy

### Build APK
```
File → Build Settings → Build
```

### Deploy su Quest 3
```bash
# Via ADB
adb install -r CloverVR.apk

# O via Meta Quest Developer Hub
```

### Permessi Richiesti
- Internet Access (per WebRTC/WebSocket)
- Microphone (opzionale, per comunicazione audio)

## Troubleshooting

### Video non si connette
1. Verificare che il rover sia sulla stessa rete WiFi
2. Controllare IP corretto
3. Verificare che `teleop_server.py` sia in esecuzione

### Latenza video alta
1. Ridurre risoluzione video (640x480)
2. Verificare qualità WiFi
3. Usare canale 5GHz se disponibile

### Controller non risponde
1. Verificare connessione WebSocket
2. Controllare logs in adb logcat
3. Verificare che Modbus sia connesso sul rover

## File Principali

```
Assets/
├── Scripts/
│   ├── CloverTeleopMain.cs    # Controller principale
│   ├── WebRTCManager.cs       # Gestione video WebRTC
│   ├── ControllerManager.cs   # Input controller Quest
│   └── StereoRenderer.cs      # Rendering stereo
├── Prefabs/
│   └── CloverTeleopRig.prefab # Prefab completo
├── Materials/
│   ├── LeftEyeMaterial.mat
│   └── RightEyeMaterial.mat
└── Scenes/
    └── TeleopScene.unity
```

## Note Sviluppo

- Il video stereo è in formato Side-by-Side (SBS)
- La latenza target è < 100ms per controllo reattivo
- I comandi vengono inviati a 20Hz
- Feedback aptico proporzionale alla velocità
