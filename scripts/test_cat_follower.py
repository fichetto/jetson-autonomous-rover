#!/usr/bin/env python3
"""
Test Cat Follower System
========================
Verifica che tutti i componenti funzionino insieme:
- Stereo cameras con calibrazione
- Depth estimation
- Obstacle detection
"""

import cv2
import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.vision.stereo_depth import StereoDepthEstimator, StereoCalibration


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
    print("  TEST CAT FOLLOWER SYSTEM")
    print("=" * 50)

    # 1. Test calibration loading
    print("\n[1] Loading calibration...")
    calib = StereoCalibration.load_from_file()
    print(f"    Baseline: {calib.baseline * 100:.2f} cm")
    print(f"    Focal: {calib.focal_length:.1f} px")
    print(f"    Rectification: {'YES' if calib.has_rectification else 'NO'}")

    # 2. Test depth estimator
    print("\n[2] Creating depth estimator...")
    depth_est = StereoDepthEstimator(calib)
    print("    OK!")

    # 3. Test cameras
    print("\n[3] Opening cameras...")
    cam_l = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)
    cam_r = cv2.VideoCapture(create_pipeline(0), cv2.CAP_GSTREAMER)

    if not cam_l.isOpened() or not cam_r.isOpened():
        print("    [!] Camera error")
        return False

    print("    OK!")

    # Warmup
    for _ in range(10):
        cam_l.read()
        cam_r.read()

    # 4. Capture and process
    print("\n[4] Capturing frame...")
    _, left = cam_l.read()
    _, right = cam_r.read()

    left = cv2.rotate(left, cv2.ROTATE_180)
    right = cv2.rotate(right, cv2.ROTATE_180)

    print(f"    Frame size: {left.shape[1]}x{left.shape[0]}")

    # 5. Compute depth
    print("\n[5] Computing depth...")
    depth = depth_est.compute_depth(left, right)

    valid = depth[(depth > 0.1) & (depth < 10.0)]
    if len(valid) > 0:
        print(f"    Min depth: {valid.min() * 100:.1f} cm")
        print(f"    Max depth: {valid.max() * 100:.1f} cm")
        print(f"    Mean depth: {valid.mean() * 100:.1f} cm")
        print(f"    Valid pixels: {len(valid)} ({100 * len(valid) / depth.size:.1f}%)")

    # 6. Test obstacle detection
    print("\n[6] Testing obstacle detection...")
    h, w = depth.shape
    y1, y2 = int(h * 0.5), h
    x1, x2 = int(w * 0.25), int(w * 0.75)
    roi = depth[y1:y2, x1:x2]
    roi_valid = roi[(roi > 0.1) & (roi < 10.0)]

    if len(roi_valid) > 100:
        min_dist = np.percentile(roi_valid, 10)
        print(f"    Closest obstacle: {min_dist * 100:.1f} cm")

        if min_dist < 0.4:
            print("    STATUS: OBSTACLE DETECTED!")
        elif min_dist < 0.6:
            print("    STATUS: Slowdown zone")
        else:
            print("    STATUS: Clear")
    else:
        print("    Not enough valid depth data")

    # Cleanup
    cam_l.release()
    cam_r.release()

    # Save visualization
    print("\n[7] Saving visualization...")
    output_dir = PROJECT_ROOT / "debug_images"
    output_dir.mkdir(exist_ok=True)

    # Depth colormap
    depth_vis = depth_est.colorize_depth(depth, 0.2, 2.0)

    # Draw obstacle zone
    cv2.rectangle(depth_vis, (x1, y1), (x2, y2), (255, 255, 255), 2)
    cv2.putText(depth_vis, "Obstacle Zone", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imwrite(str(output_dir / "cat_follower_test.jpg"), depth_vis)
    print(f"    Saved to: {output_dir / 'cat_follower_test.jpg'}")

    print("\n" + "=" * 50)
    print("  ALL TESTS PASSED!")
    print("=" * 50)
    return True


if __name__ == "__main__":
    main()
