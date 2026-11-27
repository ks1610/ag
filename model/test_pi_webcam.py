#!/usr/bin/env python3
"""Webcam -> YOLOv8 box detection test for Raspberry Pi."""
import os
import sys
import time

import cv2
from ultralytics import YOLO

MODEL_PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/ag/model/best.pt')
CAM_INDEX = int(os.environ.get('CAM', '0'))
CONF = 0.8
IMGSZ = 320
WIDTH, HEIGHT = 640, 480

model = YOLO(MODEL_PATH)
print('Classes:', model.names)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    sys.exit(f'Cannot open camera /dev/video{CAM_INDEX}')

# No display (SSH session): save annotated frames to a file instead.
headless = not os.environ.get('DISPLAY')
if headless:
    print('No DISPLAY found: writing annotated frame to /tmp/detect_latest.jpg')

last_print = 0.0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print('Frame read failed')
            break

        result = model.predict(frame, conf=CONF, imgsz=IMGSZ, verbose=False)[0]
        annotated = result.plot()

        now = time.time()
        if result.boxes is not None and len(result.boxes) > 0 and now - last_print > 1.0:
            for box in result.boxes:
                name = model.names[int(box.cls)]
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                print(f'{name} conf={float(box.conf):.2f} center=({(x1+x2)/2:.0f},{(y1+y2)/2:.0f})')
            last_print = now

        if headless:
            cv2.imwrite('/tmp/detect_latest.jpg', annotated)
            time.sleep(0.2)
        else:
            cv2.imshow('Box detection', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    cap.release()
    cv2.destroyAllWindows()
