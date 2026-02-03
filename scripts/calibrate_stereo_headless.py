#!/usr/bin/env python3
"""
Stereo Calibration HEADLESS - Niente GUI, funziona via SSH
==========================================================
Usa solo terminale, salva preview su file.
"""

import cv2
import numpy as np
import sys
import time
import select
import termios
import tty
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.025

CALIB_DIR = PROJECT_ROOT / "config"
CALIB_FILE = CALIB_DIR / "stereo_calibration.npz"
IMAGES_DIR = PROJECT_ROOT / "calibration_images"
PREVIEW_FILE = PROJECT_ROOT / "calibration_preview.jpg"


def create_pipeline(sensor_id):
    """Pipeline 640x480 @ 15fps"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )


def is_key_pressed():
    """Check if a key is pressed (non-blocking)"""
    return select.select([sys.stdin], [], [], 0)[0]


def get_key():
    """Get pressed key"""
    if is_key_pressed():
        return sys.stdin.read(1)
    return None


def capture_and_check(cam_l, cam_r, save_preview=True):
    """Cattura frame e controlla scacchiera"""
    ret_l, left = cam_l.read()
    ret_r, right = cam_r.read()

    if not ret_l or not ret_r:
        return None, None, False, False

    # Ruota 180
    left = cv2.rotate(left, cv2.ROTATE_180)
    right = cv2.rotate(right, cv2.ROTATE_180)

    # Controlla scacchiera
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
    found_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, flags)
    found_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, flags)

    # Salva preview con annotazioni
    if save_preview:
        vis_l = left.copy()
        vis_r = right.copy()

        if found_l:
            cv2.drawChessboardCorners(vis_l, CHECKERBOARD, corners_l, found_l)
        if found_r:
            cv2.drawChessboardCorners(vis_r, CHECKERBOARD, corners_r, found_r)

        color_l = (0, 255, 0) if found_l else (0, 0, 255)
        color_r = (0, 255, 0) if found_r else (0, 0, 255)
        cv2.putText(vis_l, "L:OK" if found_l else "L:NO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_l, 2)
        cv2.putText(vis_r, "R:OK" if found_r else "R:NO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_r, 2)

        combined = cv2.hconcat([vis_l, vis_r])
        cv2.imwrite(str(PREVIEW_FILE), combined)

    return left, right, found_l, found_r


def calibrate(left_images, right_images):
    """Calibrazione stereo"""
    print("\n" + "=" * 50)
    print("  CALIBRAZIONE")
    print("=" * 50)

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points, left_points, right_points = [], [], []
    image_size = None

    print(f"Analisi {len(left_images)} coppie...")

    for i, (left, right) in enumerate(zip(left_images, right_images)):
        if image_size is None:
            image_size = (left.shape[1], left.shape[0])

        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, flags)
        ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, flags)

        if ret_l and ret_r:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_l = cv2.cornerSubPix(gray_l, corners_l, (5, 5), (-1, -1), criteria)
            corners_r = cv2.cornerSubPix(gray_r, corners_r, (5, 5), (-1, -1), criteria)
            obj_points.append(objp)
            left_points.append(corners_l)
            right_points.append(corners_r)
            print(f"  [{i+1}] OK")
        else:
            print(f"  [{i+1}] Skip")

    if len(obj_points) < 8:
        print(f"[!] Solo {len(obj_points)} valide, servono 8+")
        return False

    print(f"\nCoppie valide: {len(obj_points)}")
    print("\nCalibrazione camera sinistra...")
    ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(obj_points, left_points, image_size, None, None)
    print(f"  RMS: {ret_l:.4f}")

    print("Calibrazione camera destra...")
    ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(obj_points, right_points, image_size, None, None)
    print(f"  RMS: {ret_r:.4f}")

    print("Calibrazione stereo...")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    ret_s, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
        obj_points, left_points, right_points,
        mtx_l, dist_l, mtx_r, dist_r, image_size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC
    )
    print(f"  RMS stereo: {ret_s:.4f}")

    print("Rettificazione...")
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        mtx_l, dist_l, mtx_r, dist_r, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    map1_l, map2_l = cv2.initUndistortRectifyMap(mtx_l, dist_l, R1, P1, image_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(mtx_r, dist_r, R2, P2, image_size, cv2.CV_32FC1)

    baseline = np.linalg.norm(T)
    focal = P1[0, 0]

    print(f"\n{'=' * 50}")
    print(f"  Baseline: {baseline * 100:.2f} cm")
    print(f"  Focal: {focal:.1f} px")
    print(f"  RMS: {ret_s:.4f}")
    print(f"{'=' * 50}")

    CALIB_DIR.mkdir(exist_ok=True)
    np.savez(CALIB_FILE,
        mtx_l=mtx_l, dist_l=dist_l, mtx_r=mtx_r, dist_r=dist_r,
        R=R, T=T, E=E, F=F, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
        map1_l=map1_l, map2_l=map2_l, map1_r=map1_r, map2_r=map2_r,
        baseline=baseline, focal_length=focal, image_size=image_size
    )
    print(f"\n[OK] Salvato: {CALIB_FILE}")
    return True


def main():
    print("=" * 50)
    print("  CALIBRAZIONE STEREO HEADLESS")
    print("=" * 50)
    print()
    print("Nessuna GUI - funziona via SSH/VNC senza blocchi")
    print()
    print("COMANDI (premi e invio):")
    print("  p = Preview (salva immagine di controllo)")
    print("  s = Salva coppia (se scacchiera visibile)")
    print("  c = Calibra")
    print("  q = Esci")
    print()
    print(f"Preview salvata in: {PREVIEW_FILE}")
    print("=" * 50)

    IMAGES_DIR.mkdir(exist_ok=True)

    print("\nApertura camere...")
    # Sensor IDs swapped: cameras mounted upside-down
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("[!] Errore apertura camere")
        return

    # Warmup
    for _ in range(5):
        cam_l.read()
        cam_r.read()

    print("[OK] Camere pronte")
    print("\nInserisci comando: ", end="", flush=True)

    left_images = []
    right_images = []

    try:
        while True:
            cmd = input().strip().lower()

            if cmd == 'p':
                print("Cattura preview...")
                left, right, found_l, found_r = capture_and_check(cam_l, cam_r, save_preview=True)
                if left is not None:
                    print(f"  Left: {'OK' if found_l else 'NO'}  Right: {'OK' if found_r else 'NO'}")
                    print(f"  Preview: {PREVIEW_FILE}")
                else:
                    print("  [!] Errore lettura frame")

            elif cmd == 's':
                print("Cattura e salva...")
                left, right, found_l, found_r = capture_and_check(cam_l, cam_r, save_preview=True)
                if left is not None and found_l and found_r:
                    left_images.append(left.copy())
                    right_images.append(right.copy())
                    idx = len(left_images)
                    cv2.imwrite(str(IMAGES_DIR / f"left_{idx:02d}.jpg"), left)
                    cv2.imwrite(str(IMAGES_DIR / f"right_{idx:02d}.jpg"), right)
                    print(f"  [+] Coppia {idx} salvata!")
                elif left is None:
                    print("  [!] Errore frame")
                else:
                    print(f"  [!] Scacchiera non trovata - L:{'OK' if found_l else 'NO'} R:{'OK' if found_r else 'NO'}")
                print(f"  Totale: {len(left_images)} coppie")

            elif cmd == 'c':
                if len(left_images) < 10:
                    print(f"[!] Servono 10+ immagini (hai {len(left_images)})")
                else:
                    print("Chiudo camere per calibrazione...")
                    cam_l.release()
                    cam_r.release()
                    calibrate(left_images, right_images)
                    return

            elif cmd == 'q':
                break

            else:
                print("Comandi: p=preview s=salva c=calibra q=esci")

            print("\nComando: ", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\nInterrotto")
    finally:
        cam_l.release()
        cam_r.release()

    print("[OK] Fine")


if __name__ == "__main__":
    main()
