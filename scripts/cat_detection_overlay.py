#!/usr/bin/env python3
"""
Cat Detection Overlay
=====================
Legge lo stream MJPEG dal server esistente e aggiunge detection.
NON modifica mjpeg_server.py.

Usage:
    python3 cat_detection_overlay.py

Poi apri http://localhost:8091/ per vedere lo stream con detection.
"""

import cv2
import numpy as np
import time
import threading
from flask import Flask, Response, render_template_string
import requests
import sys
import os

sys.path.insert(0, '/home/jetsonnano/autonomous-rover')

app = Flask(__name__)

# Configurazione
MJPEG_SOURCE = "http://localhost:8090/stream/left"
OUTPUT_PORT = 8091
DETECTION_INTERVAL = 3  # Detect ogni N frame

# Stato globale
frame_lock = threading.Lock()
current_frame = None
last_detections = []
detector = None


class CatDetector:
    """YOLOv8 cat detector"""
    CAT_CLASS_ID = 15  # COCO class for cat

    def __init__(self):
        self.model = None
        self._load_model()

    def _load_model(self):
        try:
            from ultralytics import YOLO
            model_path = "/home/jetsonnano/autonomous-rover/models/yolo11n.pt"
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                print(f"[OK] Loaded YOLO model: {model_path}")
            else:
                print(f"[!] Model not found: {model_path}")
        except Exception as e:
            print(f"[!] Failed to load YOLO: {e}")

    def detect(self, frame):
        """Detect cats, return list of (x1,y1,x2,y2,conf)"""
        if self.model is None:
            return []

        try:
            results = self.model(frame, verbose=False, conf=0.4)
            cats = []
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) == self.CAT_CLASS_ID:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cats.append((int(x1), int(y1), int(x2), int(y2), conf))
            return cats
        except Exception as e:
            print(f"[!] Detection error: {e}")
            return []


def draw_detections(frame, detections):
    """Disegna i box sui gatti"""
    result = frame.copy()

    for (x1, y1, x2, y2, conf) in detections:
        # Box verde
        cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Label
        label = f"Cat {conf:.0%}"
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(result, (x1, y1-h-10), (x1+w, y1), (0, 255, 0), -1)
        cv2.putText(result, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

    # Status
    status = f"Cats: {len(detections)}" if detections else "No cats"
    cv2.putText(result, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    return result


def stream_reader():
    """Legge lo stream MJPEG e applica detection"""
    global current_frame, last_detections, detector

    print(f"[*] Connecting to {MJPEG_SOURCE}...")

    frame_count = 0

    while True:
        try:
            # Apri stream MJPEG
            stream = requests.get(MJPEG_SOURCE, stream=True, timeout=10)

            if stream.status_code != 200:
                print(f"[!] Stream error: {stream.status_code}")
                time.sleep(2)
                continue

            print("[OK] Connected to MJPEG stream")

            # Buffer per leggere i frame JPEG
            buffer = bytes()

            for chunk in stream.iter_content(chunk_size=4096):
                buffer += chunk

                # Cerca inizio e fine JPEG
                start = buffer.find(b'\xff\xd8')
                end = buffer.find(b'\xff\xd9')

                if start != -1 and end != -1 and end > start:
                    # Estrai JPEG
                    jpg = buffer[start:end+2]
                    buffer = buffer[end+2:]

                    # Decodifica
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                    if frame is not None:
                        # Ruota 180° (camera capovolta)
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                        frame_count += 1

                        # Detection ogni N frame
                        if detector and frame_count % DETECTION_INTERVAL == 0:
                            last_detections = detector.detect(frame)

                        # Disegna detection
                        frame_with_overlay = draw_detections(frame, last_detections)

                        with frame_lock:
                            current_frame = frame_with_overlay

        except Exception as e:
            print(f"[!] Stream error: {e}")
            time.sleep(2)


def generate_mjpeg():
    """Genera stream MJPEG con detection overlay"""
    while True:
        with frame_lock:
            if current_frame is None:
                time.sleep(0.05)
                continue
            frame = current_frame.copy()

        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

        time.sleep(0.03)  # ~30fps


HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Cat Detection</title>
    <style>
        body { background: #1a1a1a; color: white; font-family: Arial; text-align: center; padding: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #4CAF50; border-radius: 8px; }
        .info { margin-top: 20px; color: #888; }
    </style>
</head>
<body>
    <h1>Cat Detection Overlay</h1>
    <img src="/stream">
    <div class="info">
        Source: mjpeg_server.py (port 8090)<br>
        Detection: YOLOv8n (COCO class 15 = cat)
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/stream')
def stream():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/health')
def health():
    return {"status": "ok", "detections": len(last_detections)}


def main():
    global detector

    print("="*50)
    print("  CAT DETECTION OVERLAY")
    print("="*50)
    print(f"Source: {MJPEG_SOURCE}")
    print(f"Output: http://0.0.0.0:{OUTPUT_PORT}/")
    print("="*50)

    # Inizializza detector
    print("[*] Loading cat detector...")
    detector = CatDetector()

    # Avvia thread lettore stream
    reader = threading.Thread(target=stream_reader, daemon=True)
    reader.start()

    # Aspetta che arrivi il primo frame
    print("[*] Waiting for first frame...")
    for _ in range(50):
        with frame_lock:
            if current_frame is not None:
                break
        time.sleep(0.1)

    print(f"[OK] Starting server on port {OUTPUT_PORT}")
    app.run(host='0.0.0.0', port=OUTPUT_PORT, threaded=True, debug=False)


if __name__ == "__main__":
    main()
