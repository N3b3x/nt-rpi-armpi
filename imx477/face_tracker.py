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
from scan_utils import generate_polar_scan_angles

# Face Tracker for IMX477 Camera
# Migrated from USB camera version with enhanced polar scanning

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

# Initial position (unchanged)
def initMove():
    global arm_controller
    if arm_controller:
        arm_controller.init_move()
    
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
    print("Face Tracker Init")
    load_config()
    
    # Initialize arm controller
    arm_controller = ArmController()
    initMove()

# Start game
def start():
    global __isRunning
    __isRunning = True
    print("Face Tracker Start")
    # Restore the scan thread
    threading.Thread(target=move, args=(), daemon=True).start()

def move():
    """
    Enhanced polar scan with ±180° range for face detection.
    Based on color_palletizing.py scanning pattern.
    """
    global start_greet
    global action_finish
    global arm_controller
    
    # Define scanning angles (cover ±180° range)
    angles = generate_polar_scan_angles(-180, 180, 60)  # 60° intervals for face scanning
    angle_idx = 0
    fixed_pitch = 0  # Upright pitch angle for face-level scanning
    
    while __isRunning and arm_controller:
        if not start_greet and action_finish:
            base_angle = angles[angle_idx]
            print(f"[Scan] Starting face scan at base angle: {base_angle}° (pitch {fixed_pitch}°)")

            # Rotate to base angle
            rotation_needed = base_angle - arm_controller.current_base_angle
            arm_controller.rotate_base(rotation_needed)
            time.sleep(0.15)  # Stabilization time

            # Get scan positions for this base angle
            scan_positions = arm_controller.get_scan_positions(base_angle)

            for idx, pos in enumerate(scan_positions):
                if not __isRunning:
                    break  # If stopped, exit immediately

                print(f"[Scan] Visiting position {idx+1}/{len(scan_positions)}: {pos} (pitch {fixed_pitch}°)")
                arm_controller.scan_position(pos, pitch_angle=fixed_pitch)

                # Check for face detection
                for _ in range(3):  # Check multiple times at each position
                    time.sleep(0.05)
                    if start_greet:
                        print(f"[Scan] Face detected at position {pos}")
                        break

                if start_greet:
                    # Face detected - perform greeting action
                    action_finish = False
                    
                    # Use arm controller for gripper movement
                    arm_controller.open_gripper()
                    time.sleep(0.4)
                    arm_controller.close_gripper()
                    
                    # Action when face is detected
                    action_finish = True
                    start_greet = False
                    time.sleep(0.5)
                    break

            # Move to next angle if no face detected
            if not start_greet:
                angle_idx += 1
                if angle_idx >= len(angles):
                    angle_idx = 0
                    print("[Scan] Completed full face sweep. Restarting.")
                else:
                    print(f"[Scan] Moving to next angle: {angles[angle_idx]}°")
        else:
            time.sleep(0.01)

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

def reset_camera(picam2):
    print("Resetting camera to clear color state...")
    picam2.stop()
    time.sleep(1)
    config = picam2.create_preview_configuration(
        main={"size": (1920, 1080)},
        controls={"FrameDurationLimits": (19989, 19989)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    # Force all camera controls to auto mode
    picam2.set_controls({
        "AwbEnable": True,
        "AeEnable": True,
        "AwbMode": 0
    })

if __name__ == '__main__':
    # Initialize IMX477 camera with same setup as face_detect/lab_adjust
    import os
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
    # Restore the frame display loop
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
            cv2.imshow('IMX477 Face Tracker', frame_resize)
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
    
    # Before exiting, reset the camera to clear any stuck color state
    #reset_camera(picam2)
    time.sleep(0.2) 