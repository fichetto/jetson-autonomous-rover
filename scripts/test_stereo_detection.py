#!/usr/bin/env python3
"""
Test stereo camera with cat detection
Streams side-by-side view with detection overlay via web interface
"""

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
import threading
import time
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

app = Flask(__name__)

# Global state
frame_lock = threading.Lock()
current_frame = None
detection_info = ""

# YOLOv8 detector
detector = None

def gstreamer_pipeline(sensor_id, width=640, height=480):
    """GStreamer pipeline for IMX219 - matches mjpeg_server.py"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink max-buffers=1 drop=true"
    )

class CatDetector:
    """YOLOv8 cat detector"""
    def __init__(self):
        self.model = None
        self.cat_class_id = 15  # COCO class for cat
        self.load_model()

    def load_model(self):
        try:
            from ultralytics import YOLO
            model_path = "/home/jetsonnano/autonomous-rover/models/yolo11n.pt"
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                print(f"Loaded YOLOv8 model from {model_path}")
            else:
                print(f"Model not found at {model_path}, trying to download...")
                self.model = YOLO('yolo11n.pt')
        except Exception as e:
            print(f"Failed to load YOLO: {e}")
            self.model = None

    def detect(self, frame):
        """Detect cats in frame, return list of (x, y, w, h, confidence)"""
        if self.model is None:
            return []

        try:
            results = self.model(frame, verbose=False, conf=0.3)
            cats = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    if cls == self.cat_class_id:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cats.append((int(x1), int(y1), int(x2-x1), int(y2-y1), conf))
            return cats
        except Exception as e:
            print(f"Detection error: {e}")
            return []

def capture_thread():
    """Capture from both cameras and process"""
    global current_frame, detection_info, detector

    print("Opening cameras...")
    cap0 = cv2.VideoCapture(gstreamer_pipeline(0), cv2.CAP_GSTREAMER)
    cap1 = cv2.VideoCapture(gstreamer_pipeline(1), cv2.CAP_GSTREAMER)

    if not cap0.isOpened():
        print("ERROR: Cannot open camera 0")
        return
    if not cap1.isOpened():
        print("ERROR: Cannot open camera 1")
        cap0.release()
        return

    print("Cameras opened successfully!")
    print("Initializing cat detector...")
    detector = CatDetector()

    frame_count = 0
    start_time = time.time()

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if not ret0 or not ret1:
            print("Frame capture failed")
            time.sleep(0.1)
            continue

        frame_count += 1

        # Detect cats on left camera (every 3rd frame for performance)
        cats = []
        if frame_count % 3 == 0 and detector and detector.model:
            cats = detector.detect(frame0)

        # Draw detections on left frame
        for (x, y, w, h, conf) in cats:
            cv2.rectangle(frame0, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = f"Cat {conf:.2f}"
            cv2.putText(frame0, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Create side-by-side view
        combined = np.hstack([frame0, frame1])

        # Add labels
        cv2.putText(combined, "LEFT (detection)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(combined, "RIGHT", (frame0.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # FPS counter
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        info = f"FPS: {fps:.1f} | Cats: {len(cats)}"
        cv2.putText(combined, info, (10, combined.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        with frame_lock:
            current_frame = combined.copy()
            detection_info = info

def generate_mjpeg():
    """Generate MJPEG stream"""
    while True:
        with frame_lock:
            if current_frame is None:
                time.sleep(0.1)
                continue
            frame = current_frame.copy()

        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Stereo Cat Detection</title>
    <style>
        body {
            background: #1a1a1a;
            color: white;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 { color: #4CAF50; }
        img {
            max-width: 100%;
            border: 2px solid #4CAF50;
            border-radius: 8px;
        }
        .info {
            margin-top: 20px;
            font-size: 18px;
            color: #aaa;
        }
    </style>
</head>
<body>
    <h1>🐱 Stereo Cat Detection</h1>
    <img src="/stream" alt="Stereo Stream">
    <div class="info">
        Left camera: Cat detection with YOLOv8<br>
        Right camera: Raw stereo pair<br>
        Green boxes = detected cats
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/stream')
def stream():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    # Kill any existing mjpeg server
    os.system("pkill -f mjpeg_server.py 2>/dev/null")
    time.sleep(1)

    print("=" * 50)
    print("  STEREO CAT DETECTION TEST")
    print("=" * 50)
    print("Starting capture thread...")

    capture = threading.Thread(target=capture_thread, daemon=True)
    capture.start()

    time.sleep(2)  # Wait for cameras to initialize

    print("\nStarting web server on http://0.0.0.0:8090")
    print("Open in browser to view stereo stream with cat detection")
    print("Press Ctrl+C to stop\n")

    app.run(host='0.0.0.0', port=8090, threaded=True, debug=False)

if __name__ == "__main__":
    main()
