#!/usr/bin/env python3
"""
Simple stereo camera test - captures single frames
Avoids streaming issues by doing one-shot captures
"""

import cv2
import time
import os

OUTPUT_DIR = "/home/jetsonnano/autonomous-rover/test_images"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def gstreamer_pipeline(sensor_id):
    """GStreamer pipeline for IMX219 on Jetson"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} num-buffers=10 ! "
        f"video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width=640, height=480, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink max-buffers=1 drop=true"
    )

def test_single_camera(sensor_id):
    """Test a single camera"""
    print(f"\n--- Testing camera {sensor_id} ---")

    pipeline = gstreamer_pipeline(sensor_id)
    print(f"Pipeline: {pipeline[:60]}...")

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {sensor_id}")
        return False

    print(f"Camera {sensor_id} opened, warming up...")

    # Read a few frames to warm up
    for i in range(5):
        ret, frame = cap.read()
        if ret:
            print(f"  Frame {i+1}: {frame.shape}, mean={frame.mean():.1f}")
        else:
            print(f"  Frame {i+1}: FAILED")
        time.sleep(0.1)

    # Try to get a good frame
    ret, frame = cap.read()
    if ret and frame is not None and frame.mean() > 10:
        filename = f"{OUTPUT_DIR}/camera_{sensor_id}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        success = True
    else:
        print(f"ERROR: No valid frame from camera {sensor_id}")
        success = False

    cap.release()
    time.sleep(1)  # Give the camera time to release
    return success

def main():
    print("=" * 50)
    print("  SIMPLE STEREO CAMERA TEST")
    print("=" * 50)

    # Test each camera separately
    cam0_ok = test_single_camera(0)
    cam1_ok = test_single_camera(1)

    print("\n" + "=" * 50)
    print("  RESULTS")
    print("=" * 50)
    print(f"Camera 0 (Left):  {'OK' if cam0_ok else 'FAILED'}")
    print(f"Camera 1 (Right): {'OK' if cam1_ok else 'FAILED'}")

    if cam0_ok and cam1_ok:
        print(f"\nImages saved to: {OUTPUT_DIR}/")
        print("View them with: ls -la ~/autonomous-rover/test_images/")

    return 0 if (cam0_ok and cam1_ok) else 1

if __name__ == "__main__":
    exit(main())
