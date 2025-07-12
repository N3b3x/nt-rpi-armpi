#!/usr/bin/python3
# coding=utf8
"""
Set manual white balance, exposure, and gain for the IMX477 (CSI) camera using Picamera2.
These settings must be applied each time the camera is initialized—they do NOT persist after reboot or power cycle.
"""

from picamera2 import Picamera2
import cv2
import time

# === USER SETTINGS ===
AWB_ENABLE = False  # Disable auto white balance
EXPOSURE_TIME = 10000  # Exposure time in microseconds (e.g., 10000 = 10ms)
ANALOGUE_GAIN = 1.0    # Camera gain (1.0 = no gain)

# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)

print("Available controls:")
print(picam2.camera_controls)

# Set only supported controls
picam2.set_controls({
    "AwbEnable": False,
    "AwbMode": 0,
    "ColourGains": (1.5, 1.2),
    "AeEnable": False,
    "ExposureTime": EXPOSURE_TIME,
    "AnalogueGain": ANALOGUE_GAIN,
    "Sharpness": 8.0,
    "Contrast": 1.0,
    "Saturation": 1.0,
    "Brightness": 0.0
})

picam2.start()
time.sleep(1)  # Let camera settle

print("Manual camera settings applied (where supported).")
print("Press 'q' to quit.")

while True:
    frame = picam2.capture_array()
    cv2.imshow("IMX477 Manual Settings Preview", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()

# NOTE: These settings must be set every time you start the camera. They do NOT persist after reboot or power cycle. 