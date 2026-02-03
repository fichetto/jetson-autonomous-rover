#!/usr/bin/env python3
"""
Debug Checkerboard Detection
============================
Testa varie dimensioni di scacchiera e salva immagini di debug.
"""

import cv2
import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
DEBUG_DIR = PROJECT_ROOT / "debug_images"

def create_pipeline(sensor_id):
    """Pipeline 640x480"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )

def test_checkerboard(img, name, patterns):
    """Testa vari pattern di scacchiera"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    print(f"\n{name}:")
    print(f"  Dimensione: {img.shape[1]}x{img.shape[0]}")
    print(f"  Min/Max pixel: {gray.min()}/{gray.max()}")
    print(f"  Media: {gray.mean():.1f}")

    results = []
    for pattern in patterns:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, pattern, flags)
        status = "OK" if ret else "NO"
        results.append((pattern, ret, corners))
        print(f"  Pattern {pattern[0]}x{pattern[1]}: {status}")

    return gray, results

def main():
    print("=" * 50)
    print("  DEBUG CHECKERBOARD DETECTION")
    print("=" * 50)

    DEBUG_DIR.mkdir(exist_ok=True)

    # Pattern da testare
    patterns = [
        (9, 6),   # Quello che usiamo
        (6, 9),   # Invertito
        (8, 5),   # Pattern più piccolo
        (5, 8),
        (7, 5),
        (5, 7),
    ]

    print("\nApertura camere...")
    # Sensor IDs: 1=Left, 0=Right (swapped for upside-down mounting)
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("[!] Errore apertura camere")
        return

    print("[OK] Camere aperte")

    # Warmup
    print("Warmup...")
    for _ in range(10):
        cam_l.read()
        cam_r.read()

    # Cattura
    print("\nCattura frame...")
    ret_l, left = cam_l.read()
    ret_r, right = cam_r.read()

    cam_l.release()
    cam_r.release()

    if not ret_l or not ret_r:
        print("[!] Errore lettura frame")
        return

    # Ruota 180
    left = cv2.rotate(left, cv2.ROTATE_180)
    right = cv2.rotate(right, cv2.ROTATE_180)

    # Salva raw
    cv2.imwrite(str(DEBUG_DIR / "left_raw.jpg"), left)
    cv2.imwrite(str(DEBUG_DIR / "right_raw.jpg"), right)
    print(f"\nImmagini salvate in: {DEBUG_DIR}")

    # Test pattern
    gray_l, results_l = test_checkerboard(left, "LEFT", patterns)
    gray_r, results_r = test_checkerboard(right, "RIGHT", patterns)

    # Salva grayscale
    cv2.imwrite(str(DEBUG_DIR / "left_gray.jpg"), gray_l)
    cv2.imwrite(str(DEBUG_DIR / "right_gray.jpg"), gray_r)

    # Prova con pre-processing
    print("\n--- Test con preprocessing ---")

    # Contrast enhancement
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    left_clahe = clahe.apply(gray_l)
    right_clahe = clahe.apply(gray_r)

    cv2.imwrite(str(DEBUG_DIR / "left_clahe.jpg"), left_clahe)
    cv2.imwrite(str(DEBUG_DIR / "right_clahe.jpg"), right_clahe)

    print("\nCon CLAHE (contrast enhancement):")
    for pattern in [(9, 6), (6, 9)]:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret_l, _ = cv2.findChessboardCorners(left_clahe, pattern, flags)
        ret_r, _ = cv2.findChessboardCorners(right_clahe, pattern, flags)
        print(f"  {pattern[0]}x{pattern[1]}: L={'OK' if ret_l else 'NO'} R={'OK' if ret_r else 'NO'}")

    # Blur + threshold
    left_blur = cv2.GaussianBlur(gray_l, (5, 5), 0)
    right_blur = cv2.GaussianBlur(gray_r, (5, 5), 0)

    print("\nCon blur:")
    for pattern in [(9, 6), (6, 9)]:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH
        ret_l, _ = cv2.findChessboardCorners(left_blur, pattern, flags)
        ret_r, _ = cv2.findChessboardCorners(right_blur, pattern, flags)
        print(f"  {pattern[0]}x{pattern[1]}: L={'OK' if ret_l else 'NO'} R={'OK' if ret_r else 'NO'}")

    # Crea immagine combinata
    combined = cv2.hconcat([left, right])
    cv2.imwrite(str(DEBUG_DIR / "stereo_view.jpg"), combined)

    print("\n" + "=" * 50)
    print(f"Controlla le immagini in: {DEBUG_DIR}")
    print("=" * 50)

    # Analisi della scacchiera
    found_any = False
    for pattern, ret, corners in results_l:
        if ret:
            found_any = True
            break

    if not found_any:
        print("\nPOSSIBILI CAUSE:")
        print("1. La scacchiera non è visibile o troppo lontana")
        print("2. Illuminazione insufficiente (min/max pixel vicini)")
        print("3. Pattern errato - la nostra scacchiera è 9x6 angoli interni")
        print("   (cioè 10x7 quadrati)")
        print("4. Scacchiera non completamente nel frame")
        print("5. Troppo riflesso/abbagliamento sulla superficie")

if __name__ == "__main__":
    main()
