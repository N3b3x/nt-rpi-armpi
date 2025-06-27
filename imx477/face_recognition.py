#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import numpy as np
import threading
import mediapipe as mp
from common import yaml_handle
from color_palletizing.arm_controller import ArmController
from Camera import Camera

# Face Recognition for IMX477 Camera
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

# Initial position
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
    print("Face Recognition Init")
    load_config()
    
    # Initialize arm controller
    arm_controller = ArmController()
    initMove()

# Start game
def start():
    global __isRunning
    __isRunning = True
    print("Face Recognition Start")

def move():
    """
    Enhanced polar scan with ±180° range for face detection.
    Based on color_palletizing.py scanning pattern.
    """
    global start_greet
    global action_finish
    global arm_controller
    
    # Define scanning angles (cover ±180° range)
    angles = list(range(-180, 181, 60))  # 60° intervals for face scanning
    angle_idx = 0
    fixed_pitch = -10  # Slight downward pitch angle for face detection
    
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
    # Initialize IMX477 camera
    camera = Camera(resolution=(640, 480))
    
    init()
    start()
    
    while True:
        img = camera.get_frame()
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (640, 480))
            cv2.imshow('IMX477 Face Recognition', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    
    cv2.destroyAllWindows() 