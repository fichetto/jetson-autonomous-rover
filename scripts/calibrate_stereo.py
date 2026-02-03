#!/usr/bin/env python3
"""
Stereo Camera Calibration for CLOVER Rover
==========================================
Calibra le telecamere stereo usando una scacchiera.

Uso:
1. Stampa una scacchiera 9x6 (quadrati ~2.5cm)
2. Esegui: python3 calibrate_stereo.py
3. Muovi la scacchiera davanti alle camere
4. Premi SPAZIO per catturare (servono ~15-20 immagini)
5. Premi 'c' per calibrare
6. Premi 'q' per uscire

La calibrazione viene salvata in config/stereo_calibration.npz
"""

import cv2
import numpy as np
import os
import sys
import time
from pathlib import Path

# Aggiungi project root
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Parametri scacchiera (angoli interni)
CHECKERBOARD = (9, 6)  # Colonne x Righe di angoli interni
SQUARE_SIZE = 0.025    # Dimensione quadrato in metri (2.5cm)

# Directory per salvare
CALIB_DIR = PROJECT_ROOT / "config"
CALIB_FILE = CALIB_DIR / "stereo_calibration.npz"
IMAGES_DIR = PROJECT_ROOT / "calibration_images"


def create_gstreamer_pipeline(sensor_id, width=1280, height=720, fps=30):
    """GStreamer pipeline per CSI camera"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=2"
    )


def open_cameras():
    """Apri entrambe le telecamere"""
    cameras = []
    for i in [1, 0]:  # Swapped: cameras mounted upside-down
        pipeline = create_gstreamer_pipeline(i)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            cameras.append(cap)
            print(f"[OK] Camera {i} aperta")
        else:
            print(f"[!] Camera {i} non disponibile")
    return cameras


def find_chessboard(img, checkerboard=CHECKERBOARD):
    """Trova gli angoli della scacchiera"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, checkerboard, flags)

    if ret:
        # Raffina la posizione degli angoli
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return ret, corners, gray


def draw_corners(img, corners, ret):
    """Disegna gli angoli trovati"""
    vis = img.copy()
    cv2.drawChessboardCorners(vis, CHECKERBOARD, corners, ret)
    return vis


def calibrate_stereo(left_images, right_images, image_size):
    """Esegui calibrazione stereo completa"""
    print("\n" + "="*50)
    print("  CALIBRAZIONE STEREO")
    print("="*50)

    # Punti 3D della scacchiera
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points = []  # Punti 3D
    left_points = []  # Punti 2D camera sinistra
    right_points = []  # Punti 2D camera destra

    print(f"\nAnalisi {len(left_images)} coppie di immagini...")

    for i, (left, right) in enumerate(zip(left_images, right_images)):
        ret_l, corners_l, _ = find_chessboard(left)
        ret_r, corners_r, _ = find_chessboard(right)

        if ret_l and ret_r:
            obj_points.append(objp)
            left_points.append(corners_l)
            right_points.append(corners_r)
            print(f"  [{i+1}] OK")
        else:
            print(f"  [{i+1}] Scacchiera non trovata")

    if len(obj_points) < 10:
        print(f"\n[!] Servono almeno 10 coppie valide, trovate {len(obj_points)}")
        return None

    print(f"\nCoppie valide: {len(obj_points)}")

    # 1. Calibra camera sinistra
    print("\nCalibrazione camera sinistra...")
    ret_l, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(
        obj_points, left_points, image_size, None, None
    )
    print(f"  RMS error: {ret_l:.4f}")

    # 2. Calibra camera destra
    print("Calibrazione camera destra...")
    ret_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(
        obj_points, right_points, image_size, None, None
    )
    print(f"  RMS error: {ret_r:.4f}")

    # 3. Calibrazione stereo
    print("Calibrazione stereo...")
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    ret_stereo, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
        obj_points, left_points, right_points,
        mtx_l, dist_l, mtx_r, dist_r,
        image_size, criteria=criteria, flags=flags
    )
    print(f"  RMS error stereo: {ret_stereo:.4f}")

    # 4. Calcola rettificazione
    print("Calcolo rettificazione...")
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtx_l, dist_l, mtx_r, dist_r, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )

    # 5. Calcola mappe di rettificazione
    map1_l, map2_l = cv2.initUndistortRectifyMap(
        mtx_l, dist_l, R1, P1, image_size, cv2.CV_32FC1
    )
    map1_r, map2_r = cv2.initUndistortRectifyMap(
        mtx_r, dist_r, R2, P2, image_size, cv2.CV_32FC1
    )

    # Calcola baseline
    baseline = np.linalg.norm(T)
    focal_length = P1[0, 0]  # Focal length dopo rettificazione

    print(f"\n{'='*50}")
    print("  RISULTATI CALIBRAZIONE")
    print(f"{'='*50}")
    print(f"  Baseline: {baseline*100:.2f} cm")
    print(f"  Focal length: {focal_length:.1f} px")
    print(f"  Principal point L: ({mtx_l[0,2]:.1f}, {mtx_l[1,2]:.1f})")
    print(f"  Principal point R: ({mtx_r[0,2]:.1f}, {mtx_r[1,2]:.1f})")
    print(f"  RMS Error: {ret_stereo:.4f}")

    # Salva calibrazione
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
    print(f"\n[OK] Calibrazione salvata in: {CALIB_FILE}")

    return {
        'baseline': baseline,
        'focal_length': focal_length,
        'mtx_l': mtx_l, 'mtx_r': mtx_r,
        'map1_l': map1_l, 'map2_l': map2_l,
        'map1_r': map1_r, 'map2_r': map2_r
    }


def main():
    print("="*50)
    print("  CALIBRAZIONE STEREO CLOVER")
    print("="*50)
    print(f"Scacchiera: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} angoli interni")
    print(f"Dimensione quadrato: {SQUARE_SIZE*100:.1f} cm")
    print()
    print("Comandi:")
    print("  SPAZIO = Cattura immagine")
    print("  C      = Avvia calibrazione")
    print("  R      = Reset immagini")
    print("  Q      = Esci")
    print("="*50)

    # Apri camere
    cameras = open_cameras()
    if len(cameras) < 2:
        print("[!] Servono 2 telecamere!")
        return

    # Crea directory per immagini
    IMAGES_DIR.mkdir(exist_ok=True)

    left_images = []
    right_images = []
    image_size = None

    cv2.namedWindow("Stereo Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Stereo Calibration", 1280, 480)

    frame_count = 0
    last_capture = 0

    while True:
        # Leggi frame
        ret_l, left = cameras[0].read()
        ret_r, right = cameras[1].read()

        if not ret_l or not ret_r:
            continue

        # Ruota 180° (camere capovolte)
        left = cv2.rotate(left, cv2.ROTATE_180)
        right = cv2.rotate(right, cv2.ROTATE_180)

        if image_size is None:
            image_size = (left.shape[1], left.shape[0])

        # Cerca scacchiera
        ret_l, corners_l, gray_l = find_chessboard(left)
        ret_r, corners_r, gray_r = find_chessboard(right)

        # Prepara visualizzazione
        vis_l = left.copy()
        vis_r = right.copy()

        if ret_l:
            cv2.drawChessboardCorners(vis_l, CHECKERBOARD, corners_l, ret_l)
        if ret_r:
            cv2.drawChessboardCorners(vis_r, CHECKERBOARD, corners_r, ret_r)

        # Status
        status_l = "OK" if ret_l else "NO"
        status_r = "OK" if ret_r else "NO"
        color_l = (0, 255, 0) if ret_l else (0, 0, 255)
        color_r = (0, 255, 0) if ret_r else (0, 0, 255)

        cv2.putText(vis_l, f"LEFT: {status_l}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color_l, 2)
        cv2.putText(vis_r, f"RIGHT: {status_r}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color_r, 2)
        cv2.putText(vis_l, f"Immagini: {len(left_images)}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # Combina
        combined = cv2.hconcat([vis_l, vis_r])
        cv2.imshow("Stereo Calibration", combined)

        key = cv2.waitKey(1) & 0xFF

        # SPAZIO = Cattura
        if key == ord(' '):
            if ret_l and ret_r:
                if time.time() - last_capture > 0.5:  # Debounce
                    left_images.append(left.copy())
                    right_images.append(right.copy())

                    # Salva immagini
                    idx = len(left_images)
                    cv2.imwrite(str(IMAGES_DIR / f"left_{idx:02d}.jpg"), left)
                    cv2.imwrite(str(IMAGES_DIR / f"right_{idx:02d}.jpg"), right)

                    print(f"[+] Catturata coppia {idx}")
                    last_capture = time.time()
            else:
                print("[!] Scacchiera non visibile in entrambe le camere")

        # C = Calibra
        elif key == ord('c'):
            if len(left_images) >= 10:
                result = calibrate_stereo(left_images, right_images, image_size)
                if result:
                    print("\n[OK] Calibrazione completata!")
            else:
                print(f"[!] Servono almeno 10 immagini, ne hai {len(left_images)}")

        # R = Reset
        elif key == ord('r'):
            left_images = []
            right_images = []
            print("[*] Immagini resettate")

        # Q = Esci
        elif key == ord('q'):
            break

    # Cleanup
    for cap in cameras:
        cap.release()
    cv2.destroyAllWindows()
    print("\n[OK] Calibrazione terminata")


if __name__ == "__main__":
    main()
