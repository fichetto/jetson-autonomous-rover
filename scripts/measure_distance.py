#!/usr/bin/env python3
"""
Misura Distanza - Test pratico della calibrazione stereo
========================================================
Metti un oggetto davanti alle camere e misura la distanza.
"""

import cv2
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
CALIB_FILE = PROJECT_ROOT / "config" / "stereo_calibration.npz"
OUTPUT_FILE = PROJECT_ROOT / "debug_images" / "distance_measure.jpg"


def create_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )


def measure():
    # Carica calibrazione
    calib = np.load(CALIB_FILE)
    map1_l, map2_l = calib['map1_l'], calib['map2_l']
    map1_r, map2_r = calib['map1_r'], calib['map2_r']
    baseline = float(calib['baseline'])
    focal = float(calib['focal_length'])

    # Stereo matcher
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=64,
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Apri camere
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("[!] Errore camere")
        return None

    # Warmup
    for _ in range(10):
        cam_l.read()
        cam_r.read()

    # Cattura
    _, left = cam_l.read()
    _, right = cam_r.read()
    cam_l.release()
    cam_r.release()

    # Ruota e rettifica
    left = cv2.rotate(left, cv2.ROTATE_180)
    right = cv2.rotate(right, cv2.ROTATE_180)
    left_rect = cv2.remap(left, map1_l, map2_l, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right, map1_r, map2_r, cv2.INTER_LINEAR)

    # Disparità
    gray_l = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

    # Calcola profondità
    depth = np.zeros_like(disparity)
    valid = disparity > 0
    depth[valid] = (baseline * focal) / disparity[valid]

    # Misura al centro (area 100x100 pixel)
    h, w = depth.shape
    cx, cy = w // 2, h // 2
    roi = depth[cy-50:cy+50, cx-50:cx+50]
    roi_valid = roi[roi > 0]

    if len(roi_valid) > 100:
        distance_m = np.median(roi_valid)
        distance_cm = distance_m * 100
    else:
        distance_m = 0
        distance_cm = 0

    # Crea immagine annotata
    output = left_rect.copy()

    # Disegna box centrale
    cv2.rectangle(output, (cx-50, cy-50), (cx+50, cy+50), (0, 255, 0), 2)

    # Distanza
    if distance_cm > 0:
        text = f"{distance_cm:.0f} cm"
        color = (0, 255, 0)
    else:
        text = "N/A"
        color = (0, 0, 255)

    cv2.putText(output, text, (cx-40, cy-60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
    cv2.putText(output, "Centro", (cx-30, cy+80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Info
    cv2.putText(output, f"Baseline: {baseline*100:.1f}cm  Focal: {focal:.0f}px",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Salva
    Path(OUTPUT_FILE).parent.mkdir(exist_ok=True)
    cv2.imwrite(str(OUTPUT_FILE), output)

    return distance_cm


if __name__ == "__main__":
    print("=" * 40)
    print("  MISURA DISTANZA")
    print("=" * 40)
    print("\nPosiziona un oggetto al CENTRO")
    print("dell'inquadratura delle camere.\n")

    input("Premi INVIO quando pronto...")

    print("\nMisurazione in corso...")
    dist = measure()

    if dist and dist > 0:
        print(f"\n>>> DISTANZA: {dist:.0f} cm <<<\n")
    else:
        print("\n[!] Impossibile misurare - oggetto troppo vicino/lontano\n")

    print(f"Immagine salvata: {OUTPUT_FILE}")
