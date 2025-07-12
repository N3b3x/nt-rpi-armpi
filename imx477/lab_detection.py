#!/usr/bin/python3
from picamera2 import Picamera2
import cv2
import numpy as np

# LAB thresholds (example)
lab_data = {
    'red': {
        'min': [130, 200, 180],
        'max': [140, 215, 210]
    },
    'green': {
        'min': [190, 60, 150],
        'max': [200, 70, 160]
    },
    'blue': {
        'min': [175, 95, 90],
        'max': [185, 110, 105]
    }
}

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

print("✅ Starting live LAB detection with PiCamera2...")

while True:
    # Capture frame
    frame = picam2.capture_array()

    # Confirm data shape and type
    if frame is None:
        print("❌ No frame received!")
        continue

    # frame from picamera2 is RGB already.
    frame_rgb = frame  # Already RGB, no need to convert

    # For OpenCV, convert to BGR for display
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    # LAB conversion
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

    # Show center LAB value
    h, w = frame_lab.shape[:2]
    center_lab = frame_lab[h//2, w//2]
    print("Center LAB:", center_lab)

    # Create masks
    red_mask = cv2.inRange(frame_lab, tuple(lab_data['red']['min']), tuple(lab_data['red']['max']))
    green_mask = cv2.inRange(frame_lab, tuple(lab_data['green']['min']), tuple(lab_data['green']['max']))
    blue_mask = cv2.inRange(frame_lab, tuple(lab_data['blue']['min']), tuple(lab_data['blue']['max']))

    # Overlay text
    cv2.putText(frame_bgr, f"Center LAB: {center_lab}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame_bgr, "Press 'q' to quit", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Show images
    cv2.imshow('Original (BGR, swapped)', frame_bgr)
    cv2.imshow('Red Mask', red_mask)
    cv2.imshow('Green Mask', green_mask)
    cv2.imshow('Blue Mask', blue_mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        print("✅ Quitting.")
        break

cv2.destroyAllWindows()
picam2.stop()
