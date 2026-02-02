#!/usr/bin/env python3
"""
Cat Tracker - Ruota il rover per centrare il gatto nell'inquadratura

Usa:
- mjpeg_server.py (porta 8090) per detection
- RoverService (porta 8081) per controllo motori
"""

import requests
import time
import sys

# Config
MJPEG_SERVER = "http://localhost:8090"
ROVER_SERVICE = "http://localhost:8082"
FRAME_WIDTH = 1280  # Larghezza frame camera
DEAD_ZONE = 80      # Pixel dal centro dove non ruotiamo
KP = 0.0005         # Gain proporzionale per rotazione
MAX_WZ = 0.12       # Max velocità angolare (lenta)


def connect_rover():
    """Connetti a Arduino via RoverService"""
    try:
        r = requests.post(f"{ROVER_SERVICE}/connect", timeout=5)
        if r.ok:
            print("[OK] Connesso ad Arduino")
            return True
        else:
            print(f"[!] Errore connessione: {r.text}")
            return False
    except Exception as e:
        print(f"[!] RoverService non raggiungibile: {e}")
        return False


def set_mode(mode: str):
    """Imposta modalità rover"""
    try:
        r = requests.post(f"{ROVER_SERVICE}/mode", json={"mode": mode}, timeout=2)
        return r.ok
    except:
        return False


def rotate(wz: float):
    """Invia comando rotazione"""
    try:
        requests.post(f"{ROVER_SERVICE}/move",
                     json={"vx": 0, "vy": 0, "wz": wz},
                     timeout=0.5)
    except:
        pass


def stop():
    """Ferma motori"""
    try:
        requests.post(f"{ROVER_SERVICE}/stop", timeout=0.5)
    except:
        pass


def get_detections():
    """Ottieni detection dal server MJPEG"""
    try:
        r = requests.get(f"{MJPEG_SERVER}/detection/status", timeout=2)
        if r.ok:
            return r.json()
    except Exception as e:
        print(f"[!] Errore detection: {e}")
    return {"enabled": False, "cats": 0, "detections": []}


def main():
    print("=" * 50)
    print("  CAT TRACKER - Centra gatto nell'inquadratura")
    print("=" * 50)
    print(f"Detection: {MJPEG_SERVER}")
    print(f"Rover: {ROVER_SERVICE}")
    print(f"Dead zone: ±{DEAD_ZONE}px dal centro")
    print("Premi Ctrl+C per fermare")
    print("=" * 50)

    # Verifica detection attiva
    status = get_detections()
    if not status.get("enabled"):
        print("[*] Attivo detection...")
        requests.post(f"{MJPEG_SERVER}/detection/toggle")
        time.sleep(1)

    # Connetti rover
    if not connect_rover():
        print("[!] Impossibile connettersi. Avviare RoverService?")
        sys.exit(1)

    # Imposta modalità autonomous
    set_mode("autonomous")
    print("[OK] Modalità: autonomous")

    center_x = FRAME_WIDTH // 2
    tracking = False

    try:
        errors = 0
        while True:
            status = get_detections()

            # Riattiva detection se disabilitata
            if not status.get("enabled"):
                print("[!] Detection disabilitata, riattivo...")
                requests.post(f"{MJPEG_SERVER}/detection/toggle", timeout=2)
                time.sleep(1)
                continue

            if status.get("cats", 0) > 0:
                errors = 0  # Reset errori
                # Prendi il gatto più grande (più vicino)
                detections = status.get("detections", [])
                if detections:
                    # Formato: (x1, y1, x2, y2, conf)
                    biggest = max(detections, key=lambda d: (d[2]-d[0]) * (d[3]-d[1]))
                    x1, y1, x2, y2, conf = biggest

                    cat_center_x = (x1 + x2) // 2
                    error = cat_center_x - center_x

                    if not tracking:
                        print(f"[CAT] Gatto rilevato! conf={conf:.0%}")
                        tracking = True

                    # Se fuori dalla dead zone, ruota
                    if abs(error) > DEAD_ZONE:
                        # Errore positivo = gatto a destra = ruota a destra (wz negativo)
                        wz = -KP * error
                        wz = max(-MAX_WZ, min(MAX_WZ, wz))
                        rotate(wz)
                        direction = "←" if wz > 0 else "→"
                        print(f"  Offset: {error:+4d}px | Ruoto {direction} wz={wz:+.2f}")
                    else:
                        stop()
                        print(f"  Centrato! (offset: {error:+d}px)")
            else:
                if tracking:
                    print("[--] Gatto perso, fermo")
                    tracking = False
                stop()

            time.sleep(0.05)  # 20 Hz loop

    except KeyboardInterrupt:
        print("\n[*] Arresto...")
    finally:
        stop()
        set_mode("disabled")
        print("[OK] Tracker fermato")


if __name__ == "__main__":
    main()
