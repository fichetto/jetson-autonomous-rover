#!/usr/bin/env python3
"""
Stereo Calibration LITE - Versione leggera per Jetson Orin
==========================================================
Risoluzione ridotta e detection solo su richiesta.
"""

import cv2
import numpy as np
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Parametri scacchiera
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.025  # 2.5cm

CALIB_DIR = PROJECT_ROOT / "config"
CALIB_FILE = CALIB_DIR / "stereo_calibration.npz"
IMAGES_DIR = PROJECT_ROOT / "calibration_images"


def create_pipeline(sensor_id):
    """Pipeline leggera 640x480"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )


def main():
    print("=" * 50)
    print("  CALIBRAZIONE STEREO LITE")
    print("=" * 50)
    print("Comandi:")
    print("  D = Detect scacchiera (controlla se visibile)")
    print("  SPAZIO = Cattura coppia")
    print("  C = Calibra (dopo 10+ catture)")
    print("  Q = Esci")
    print("=" * 50)

    # Apri camere
    print("Apertura camere...")
    # Sensor IDs swapped: cameras mounted upside-down
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("[!] Errore apertura camere")
        return

    print("[OK] Camere aperte")

    IMAGES_DIR.mkdir(exist_ok=True)

    left_images = []
    right_images = []
    last_detect_ok = [False, False]

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration", 1280, 480)

    frame_skip = 0

    while True:
        ret_l, left = cam_l.read()
        ret_r, right = cam_r.read()

        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

        # Ruota 180
        left = cv2.rotate(left, cv2.ROTATE_180)
        right = cv2.rotate(right, cv2.ROTATE_180)

        # Visualizzazione semplice (no detection continua)
        vis_l = left.copy()
        vis_r = right.copy()

        # Status
        status_l = "OK" if last_detect_ok[0] else "--"
        status_r = "OK" if last_detect_ok[1] else "--"
        color_l = (0, 255, 0) if last_detect_ok[0] else (128, 128, 128)
        color_r = (0, 255, 0) if last_detect_ok[1] else (128, 128, 128)

        cv2.putText(vis_l, f"L:{status_l}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_l, 2)
        cv2.putText(vis_r, f"R:{status_r}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_r, 2)
        cv2.putText(vis_l, f"N={len(left_images)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        cv2.putText(vis_l, "D=detect SPACE=capture C=calib Q=quit", (10, 470),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

        combined = cv2.hconcat([vis_l, vis_r])
        cv2.imshow("Calibration", combined)

        key = cv2.waitKey(30) & 0xFF

        # D = Detect (controlla scacchiera)
        if key == ord('d'):
            print("Detecting...")
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
            ret_l, _ = cv2.findChessboardCorners(gray_l, CHECKERBOARD, flags)
            ret_r, _ = cv2.findChessboardCorners(gray_r, CHECKERBOARD, flags)

            last_detect_ok = [ret_l, ret_r]
            print(f"  Left: {'OK' if ret_l else 'NO'}  Right: {'OK' if ret_r else 'NO'}")

        # SPAZIO = Cattura
        elif key == ord(' '):
            print("Cattura e verifica...")
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, flags)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, flags)

            if ret_l and ret_r:
                left_images.append(left.copy())
                right_images.append(right.copy())
                idx = len(left_images)
                cv2.imwrite(str(IMAGES_DIR / f"left_{idx:02d}.jpg"), left)
                cv2.imwrite(str(IMAGES_DIR / f"right_{idx:02d}.jpg"), right)
                print(f"[+] Coppia {idx} salvata!")
                last_detect_ok = [True, True]
            else:
                print(f"[!] Scacchiera non trovata - L:{'OK' if ret_l else 'NO'} R:{'OK' if ret_r else 'NO'}")
                last_detect_ok = [ret_l, ret_r]

        # C = Calibra
        elif key == ord('c'):
            if len(left_images) < 10:
                print(f"[!] Servono almeno 10 immagini (hai {len(left_images)})")
            else:
                print("\nAvvio calibrazione...")
                cam_l.release()
                cam_r.release()
                cv2.destroyAllWindows()

                calibrate(left_images, right_images)
                return

        # Q = Esci
        elif key == ord('q'):
            break

    cam_l.release()
    cam_r.release()
    cv2.destroyAllWindows()


def calibrate(left_images, right_images):
    """Calibrazione offline"""
    print("\n" + "=" * 50)
    print("  CALIBRAZIONE IN CORSO")
    print("=" * 50)

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points = []
    left_points = []
    right_points = []
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
        print(f"\n[!] Solo {len(obj_points)} coppie valide, servono almeno 8")
        return

    print(f"\nCoppie valide: {len(obj_points)}")

    # Calibra singole camere
    print("\nCamera sinistra...")
    ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(obj_points, left_points, image_size, None, None)
    print(f"  RMS: {ret_l:.4f}")

    print("Camera destra...")
    ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(obj_points, right_points, image_size, None, None)
    print(f"  RMS: {ret_r:.4f}")

    # Calibrazione stereo
    print("Stereo...")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    ret_s, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
        obj_points, left_points, right_points,
        mtx_l, dist_l, mtx_r, dist_r,
        image_size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC
    )
    print(f"  RMS stereo: {ret_s:.4f}")

    # Rettificazione
    print("Rettificazione...")
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtx_l, dist_l, mtx_r, dist_r, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(mtx_l, dist_l, R1, P1, image_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(mtx_r, dist_r, R2, P2, image_size, cv2.CV_32FC1)

    baseline = np.linalg.norm(T)
    focal_length = P1[0, 0]

    print(f"\n{'=' * 50}")
    print("  RISULTATI")
    print(f"{'=' * 50}")
    print(f"  Baseline: {baseline * 100:.2f} cm")
    print(f"  Focal length: {focal_length:.1f} px")
    print(f"  RMS Error: {ret_s:.4f}")

    # Salva
    CALIB_DIR.mkdir(exist_ok=True)
    np.savez(CALIB_FILE,
        mtx_l=mtx_l, dist_l=dist_l,
        mtx_r=mtx_r, dist_r=dist_r,
        R=R, T=T, E=E, F=F,
        R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
        map1_l=map1_l, map2_l=map2_l,
        map1_r=map1_r, map2_r=map2_r,
        baseline=baseline, focal_length=focal_length,
        image_size=image_size
    )
    print(f"\n[OK] Salvato in: {CALIB_FILE}")


if __name__ == "__main__":
    main()
