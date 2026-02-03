#!/usr/bin/env python3
"""
Test CLAHE Enhancement and Cat Detection
=========================================
Quick test to verify CLAHE preprocessing is helping cat detection.
Saves comparison images and shows detection results.
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.vision.cat_follower import CatDetector

# COCO class names for common objects
COCO_NAMES = {
    0: 'person', 15: 'cat', 16: 'dog', 17: 'horse',
    56: 'chair', 57: 'couch', 58: 'potted_plant', 59: 'bed',
    60: 'dining_table', 62: 'tv', 63: 'laptop'
}


def create_pipeline(sensor_id):
    """Create GStreamer pipeline for camera"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1"
    )


def main():
    print("=" * 60)
    print("  CLAHE Enhancement Test for Cat Detection")
    print("=" * 60)

    # Create output directory
    output_dir = PROJECT_ROOT / "debug_images"
    output_dir.mkdir(exist_ok=True)

    # Initialize detector WITH enhancement
    print("\n[1] Initializing detector with CLAHE enhancement...")
    detector_enhanced = CatDetector(
        model_path=str(PROJECT_ROOT / "models" / "yolo11n.pt"),
        conf_threshold=0.35,
        enhance_image=True,
        debug_dir=str(output_dir)
    )

    # Initialize detector WITHOUT enhancement for comparison
    print("[2] Initializing detector without enhancement...")
    detector_raw = CatDetector(
        model_path=str(PROJECT_ROOT / "models" / "yolo11n.pt"),
        conf_threshold=0.35,
        enhance_image=False
    )

    # Open camera (use left camera - sensor 1)
    print("\n[3] Opening camera...")
    cap = cv2.VideoCapture(create_pipeline(1), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[!] Failed to open camera")
        return

    print("[OK] Camera opened")

    # Warmup
    print("[4] Warming up...")
    for _ in range(15):
        cap.read()

    print("\n[5] Running detection comparison...")
    print("    (Press Ctrl+C to stop)")
    print("-" * 60)

    frame_count = 0
    enhanced_detections = 0
    raw_detections = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Rotate if cameras are upside down
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Detect with enhanced image
            cats_enhanced = detector_enhanced.detect_cats(frame)

            # Detect with raw image
            cats_raw = detector_raw.detect_cats(frame)

            frame_count += 1

            if cats_enhanced:
                enhanced_detections += 1
            if cats_raw:
                raw_detections += 1

            # Log every 30 frames
            if frame_count % 30 == 0:
                print(f"Frame {frame_count}: Enhanced={len(cats_enhanced)} cats, Raw={len(cats_raw)} cats")
                print(f"  Total: Enhanced found cats in {enhanced_detections}/{frame_count} frames, "
                      f"Raw in {raw_detections}/{frame_count} frames")

            # Save comparison image every 100 frames
            if frame_count % 100 == 1:
                # Create CLAHE enhanced version
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                l_enhanced = clahe.apply(l)
                lab_enhanced = cv2.merge([l_enhanced, a, b])
                enhanced_frame = cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)

                # Draw detections on both
                frame_vis = frame.copy()
                enhanced_vis = enhanced_frame.copy()

                for cat in cats_raw:
                    x1, y1, x2, y2 = cat['bbox']
                    cv2.rectangle(frame_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame_vis, f"Cat {cat['confidence']:.2f}", (x1, y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                for cat in cats_enhanced:
                    x1, y1, x2, y2 = cat['bbox']
                    cv2.rectangle(enhanced_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(enhanced_vis, f"Cat {cat['confidence']:.2f}", (x1, y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Labels
                cv2.putText(frame_vis, f"RAW ({len(cats_raw)} cats)", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(enhanced_vis, f"CLAHE ({len(cats_enhanced)} cats)", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Side by side
                comparison = np.hstack([frame_vis, enhanced_vis])
                cv2.imwrite(str(output_dir / f"clahe_comparison_{frame_count}.jpg"), comparison)
                print(f"  [Saved comparison image: frame {frame_count}]")

            # Small delay to avoid overloading
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n" + "-" * 60)
        print("Stopped by user")

    finally:
        cap.release()

        # Summary
        print("\n" + "=" * 60)
        print("  SUMMARY")
        print("=" * 60)
        print(f"Total frames: {frame_count}")
        print(f"Enhanced detection rate: {enhanced_detections}/{frame_count} "
              f"({100*enhanced_detections/max(1,frame_count):.1f}%)")
        print(f"Raw detection rate: {raw_detections}/{frame_count} "
              f"({100*raw_detections/max(1,frame_count):.1f}%)")

        if enhanced_detections > raw_detections:
            print("\n[+] CLAHE enhancement IMPROVED detection!")
        elif enhanced_detections < raw_detections:
            print("\n[-] CLAHE enhancement might not be helping")
        else:
            print("\n[=] No significant difference")

        print(f"\nComparison images saved to: {output_dir}")


if __name__ == "__main__":
    main()
