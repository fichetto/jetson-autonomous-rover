#!/usr/bin/env python3
"""
Hardware Test Suite for Jetson Orin Nano Autonomous Rover
Tests GPU, cameras, USB devices after potential short circuit damage
"""

import sys
import subprocess
import cv2
import numpy as np

def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}")

def test_cuda():
    """Test CUDA availability"""
    print_header("CUDA Test")
    try:
        import torch
        print(f"✓ PyTorch version: {torch.__version__}")
        print(f"✓ CUDA available: {torch.cuda.is_available()}")

        if torch.cuda.is_available():
            print(f"✓ CUDA version: {torch.version.cuda}")
            print(f"✓ GPU device: {torch.cuda.get_device_name(0)}")
            print(f"✓ GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB")

            # Test GPU computation
            x = torch.rand(1000, 1000).cuda()
            y = torch.rand(1000, 1000).cuda()
            z = torch.matmul(x, y)
            print(f"✓ GPU computation test: PASSED")

            del x, y, z
            torch.cuda.empty_cache()
            return True
        else:
            print("✗ CUDA not available")
            return False
    except Exception as e:
        print(f"✗ CUDA test failed: {e}")
        return False

def test_camera(device_id):
    """Test camera device"""
    print(f"\n  Testing /dev/video{device_id}...")
    try:
        cap = cv2.VideoCapture(device_id)

        if not cap.isOpened():
            print(f"  ✗ Cannot open /dev/video{device_id}")
            return False

        # Get camera properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))

        print(f"  ✓ Camera opened: {width}x{height} @ {fps}fps")

        # Try to read a frame
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  ✓ Frame captured: {frame.shape}")

            # Check if frame is not black (all zeros)
            mean_val = np.mean(frame)
            if mean_val > 10:
                print(f"  ✓ Frame valid (mean intensity: {mean_val:.1f})")
            else:
                print(f"  ⚠ Frame might be black (mean intensity: {mean_val:.1f})")
        else:
            print(f"  ✗ Cannot read frame")
            cap.release()
            return False

        cap.release()
        return True

    except Exception as e:
        print(f"  ✗ Camera test failed: {e}")
        return False

def test_cameras():
    """Test all video devices"""
    print_header("Camera Test")
    results = []
    for i in range(2):
        results.append(test_camera(i))

    return any(results)

def test_usb_devices():
    """List USB devices"""
    print_header("USB Devices")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        print(result.stdout)
        return True
    except Exception as e:
        print(f"✗ USB test failed: {e}")
        return False

def test_opencv():
    """Test OpenCV build info"""
    print_header("OpenCV Test")
    try:
        print(f"✓ OpenCV version: {cv2.__version__}")
        build_info = cv2.getBuildInformation()

        # Check for CUDA support
        if "CUDA" in build_info:
            print("✓ OpenCV built with CUDA support")
        else:
            print("⚠ OpenCV built WITHOUT CUDA support")

        # Check for GStreamer support
        if "GStreamer" in build_info:
            print("✓ OpenCV built with GStreamer support")
        else:
            print("⚠ OpenCV built WITHOUT GStreamer support")

        return True
    except Exception as e:
        print(f"✗ OpenCV test failed: {e}")
        return False

def test_serial_ports():
    """Check for serial devices"""
    print_header("Serial Devices Test")
    import glob

    patterns = ['/dev/ttyACM*', '/dev/ttyUSB*', '/dev/ttyTHS*']
    found_devices = []

    for pattern in patterns:
        devices = glob.glob(pattern)
        found_devices.extend(devices)

    if found_devices:
        print("✓ Serial devices found:")
        for dev in found_devices:
            print(f"  - {dev}")
        return True
    else:
        print("✗ No serial devices found (Arduino not connected?)")
        return False

def main():
    print("\n" + "="*60)
    print("  JETSON ORIN NANO - HARDWARE DIAGNOSTIC TEST")
    print("  Post Short-Circuit Check")
    print("="*60)

    results = {
        "OpenCV": test_opencv(),
        "CUDA": test_cuda(),
        "Cameras": test_cameras(),
        "USB": test_usb_devices(),
        "Serial": test_serial_ports()
    }

    print_header("TEST SUMMARY")

    all_passed = True
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {test_name:.<40} {status}")
        if not result:
            all_passed = False

    print("\n" + "="*60)

    if all_passed:
        print("  ✓ ALL TESTS PASSED - Hardware appears OK")
    else:
        print("  ⚠ SOME TESTS FAILED - Check hardware connections")

    print("="*60 + "\n")

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
