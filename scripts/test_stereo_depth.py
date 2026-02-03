#!/usr/bin/env python3
"""
Test Stereo Depth Estimation
============================
Testa la calibrazione stereo calcolando la mappa di profondità.
"""

import cv2
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
CALIB_FILE = PROJECT_ROOT / "config" / "stereo_calibration.npz"
OUTPUT_DIR = PROJECT_ROOT / "debug_images"


def create_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )


def main():
    print("=" * 50)
    print("  TEST STEREO DEPTH")
    print("=" * 50)

    # Carica calibrazione
    if not CALIB_FILE.exists():
        print(f"[!] Calibrazione non trovata: {CALIB_FILE}")
        return

    calib = np.load(CALIB_FILE)
    print(f"[OK] Calibrazione caricata")
    print(f"     Baseline: {float(calib['baseline']) * 100:.2f} cm")
    print(f"     Focal: {float(calib['focal_length']):.1f} px")

    map1_l = calib['map1_l']
    map2_l = calib['map2_l']
    map1_r = calib['map1_r']
    map2_r = calib['map2_r']
    baseline = float(calib['baseline'])
    focal = float(calib['focal_length'])

    # Stereo matcher
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=64,  # Deve essere multiplo di 16
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Apri camere
    print("\nApertura camere...")
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("[!] Errore apertura camere")
        return

    print("[OK] Camere aperte")

    # Warmup
    for _ in range(10):
        cam_l.read()
        cam_r.read()

    # Cattura
    print("\nCattura frame...")
    _, left = cam_l.read()
    _, right = cam_r.read()

    cam_l.release()
    cam_r.release()

    # Ruota 180
    left = cv2.rotate(left, cv2.ROTATE_180)
    right = cv2.rotate(right, cv2.ROTATE_180)

    # Rettifica
    print("Rettificazione...")
    left_rect = cv2.remap(left, map1_l, map2_l, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right, map1_r, map2_r, cv2.INTER_LINEAR)

    # Calcola disparità
    print("Calcolo disparità...")
    gray_l = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)

    disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

    # Calcola profondità
    print("Calcolo profondità...")
    depth = np.zeros_like(disparity)
    valid = disparity > 0
    depth[valid] = (baseline * focal) / disparity[valid]

    # Statistiche
    valid_depth = depth[valid]
    if len(valid_depth) > 0:
        print(f"\n=== RISULTATI ===")
        print(f"  Depth min: {valid_depth.min() * 100:.1f} cm")
        print(f"  Depth max: {valid_depth.max() * 100:.1f} cm")
        print(f"  Depth media: {valid_depth.mean() * 100:.1f} cm")
        print(f"  Pixel validi: {np.sum(valid)} / {disparity.size} ({100*np.sum(valid)/disparity.size:.1f}%)")

    # Visualizzazione
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Disparity colormap
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_color = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_JET)

    # Depth colormap (0-2m range)
    depth_clip = np.clip(depth, 0, 2.0)  # Max 2 metri
    depth_vis = (depth_clip / 2.0 * 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

    # Salva immagini
    cv2.imwrite(str(OUTPUT_DIR / "stereo_left_raw.jpg"), left)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_right_raw.jpg"), right)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_left_rect.jpg"), left_rect)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_right_rect.jpg"), right_rect)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_disparity.jpg"), disp_color)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_depth.jpg"), depth_color)

    # Immagine combinata con linee epipolari
    combined_rect = cv2.hconcat([left_rect, right_rect])
    for y in range(0, combined_rect.shape[0], 30):
        cv2.line(combined_rect, (0, y), (combined_rect.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite(str(OUTPUT_DIR / "stereo_rectified.jpg"), combined_rect)

    # Overlay depth su immagine
    overlay = left_rect.copy()
    mask = depth > 0
    overlay[mask] = cv2.addWeighted(overlay, 0.5, depth_color, 0.5, 0)[mask]
    cv2.imwrite(str(OUTPUT_DIR / "stereo_overlay.jpg"), overlay)

    print(f"\n[OK] Immagini salvate in: {OUTPUT_DIR}")
    print("  - stereo_rectified.jpg  (verifica allineamento)")
    print("  - stereo_disparity.jpg  (mappa disparità)")
    print("  - stereo_depth.jpg      (mappa profondità)")
    print("  - stereo_overlay.jpg    (depth overlay)")


if __name__ == "__main__":
    main()
