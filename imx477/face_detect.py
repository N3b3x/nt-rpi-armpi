#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')
sys.path.append('/home/pi/ArmPi_mini/')
import cv2
import time
import numpy as np
import threading
import mediapipe as mp
import lab_auto_calibration  # <-- Add this import
from common import yaml_handle
from arm_controller import ArmController
from Camera import Camera

# Face Detection for IMX477 Camera
# Migrated from USB camera version using ArmController

debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
# Import face detection module
Face = mp.solutions.face_detection
# Customize face detection method, minimum face detection confidence 0.8
faceDetection = Face.FaceDetection(min_detection_confidence=0.8)

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

# Initialize arm controller
arm_controller = None

# Initial position (disabled for face detection)
def initMove():
    global arm_controller
    if arm_controller:
        # Don't move arm for face detection
        pass
    

start_greet = False
action_finish = True

# Reset variables
def reset():
    global start_greet
    global action_finish

    start_greet = False
    action_finish = True
    initMove()  

__isRunning = False

# Initialize app
def init():
    global arm_controller
    print("Face Detection Init")
    load_config()
    
    # Initialize arm controller for buzzer access only (no movement)
    arm_controller = ArmController()
    # Don't call initMove() - we don't want arm movement for face detection

# Start game
def start():
    global __isRunning
    __isRunning = True
    print("Face Detection Start")

def move():
    global start_greet
    global action_finish
    global arm_controller
    
    while True:
        if __isRunning and arm_controller:
            if start_greet:
                start_greet = False
                action_finish = False
                
                # Use arm controller's board for buzzer
                arm_controller.board.set_buzzer(1900, 0.1, 0.3, 2)  # Set buzzer to sound for 0.1s
                time.sleep(0.9)
                
                action_finish = True
                
            else:
                time.sleep(0.2)
        else:
            time.sleep(0.2)
    

# Run sub-thread
threading.Thread(target=move, args=(), daemon=True).start()

size = (640, 480)  # Updated for IMX477 resolution

def run(img):
    global __isRunning, area
    global center_x, center_y
    global start_greet
    global action_finish
    
    if not __isRunning:   # Check if game is started, if not return original image
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
     
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    results = faceDetection.process(imgRGB)  # Process each frame with face detection

    if results.detections:  # If faces are detected

        for index, detection in enumerate(results.detections):  # Return face index and keypoint coordinates
            scores = list(detection.score)
            if scores and scores[0] > 0.7:
                bboxC = detection.location_data.relative_bounding_box  # Set bounding box for all boxes' xywh and keypoint info
                
                # Convert bounding box coordinates from relative to pixel coordinates
                bbox = (
                    int(bboxC.xmin * img_w),
                    int(bboxC.ymin * img_h),
                    int(bboxC.width * img_w),
                    int(bboxC.height * img_h)
                )
                
                cv2.rectangle(img, bbox, (0, 255, 0), 2)  # Draw rectangle on each frame
                
                # Get recognition box info, xy is upper left corner coordinates
                x, y, w, h = bbox
                center_x = int(x + (w / 2))
                center_y = int(y + (h / 2))
                area = int(w * h)
                if action_finish:
                    start_greet = True

    else:
        center_x, center_y, area = -1, -1, 0
            
    return img

if __name__ == '__main__':
    # Initialize IMX477 camera with same setup as lab_adjust.py
    from picamera2 import Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (1920, 1080)},
        controls={"FrameDurationLimits": (19989, 19989)}  # ~50 FPS
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    # Force all camera controls to auto mode
    picam2.set_controls({
        "AwbEnable": True,
        "AeEnable": True,
        "AwbMode": 0
    })

    # Undistort support
    undistort_enabled = False
    K, D = None, None
    if os.path.exists('calibration_data.npz'):
        K, D = lab_auto_calibration.load_calibration_data('calibration_data.npz')

    init()
    start()
    
    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if undistort_enabled and K is not None and D is not None:
                frame_bgr = lab_auto_calibration.undistort_frame(frame_bgr, K, D)
            Frame = run(frame_bgr)
            frame_resize = cv2.resize(Frame, (640, 480))
            # Overlay key info
            cv2.putText(frame_resize, '[u] Toggle undistort', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(frame_resize, '[q] Quit', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.imshow('IMX477 Face Detection', frame_resize)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('u'):
                undistort_enabled = not undistort_enabled
                print(f"Undistortion {'enabled' if undistort_enabled else 'disabled'}")
            elif key == ord('q') or key == 27:
                print("Quitting...")
                break
        else:
            time.sleep(0.01)
    
    cv2.destroyAllWindows() 