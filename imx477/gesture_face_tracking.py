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
import math
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import lab_auto_calibration
from common import yaml_handle
from arm_controller import ArmController
from Camera import Camera
from scan_utils import generate_polar_scan_angles

# Import our modular components
from tracking_components.gesture_detector import GestureDetector
from tracking_components.face_detector import FaceDetector
from tracking_components.motion_detector import MotionDetector
from tracking_components.depth_estimator import DepthEstimator
from tracking_components.zoom_controller import ZoomController

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Initialize modular components
gesture_detector = GestureDetector()
face_detector = FaceDetector()
motion_detector = MotionDetector()
depth_estimator = DepthEstimator()
zoom_controller = ZoomController()

# Initialize PID controllers for smooth tracking
x_pid = PID.PID(P=0.26, I=0.05, D=0.008)
y_pid = PID.PID(P=0.012, I=0, D=0.000)
z_pid = PID.PID(P=0.003, I=0, D=0)

# Initial arm position
servo1 = 1500
x_dis = 1500
y_dis = 6
Z_DIS = 18
z_dis = Z_DIS

# Initialize arm controller
arm_controller = None

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def initMove():
    global arm_controller
    if arm_controller:
        arm_controller.init_move_face_scan()

def set_rgb(color):
    if arm_controller:
        arm_controller.set_rgb(color)

def control_spotlight(on=True, brightness=100, color=(255, 255, 255)):
    """Control the spotlight mounted on the arm"""
    motion_detector.control_spotlight(on, brightness, color)

def detect_motion(frame):
    """Detect motion in the frame"""
    return motion_detector.detect_motion(frame)

def check_motion_timeout():
    """Check if motion has timed out and turn off spotlight if needed"""
    motion_detector.check_motion_timeout()

_stop = False
__isRunning = False
detection_mode = 'face'  # 'face' or 'hand'
current_gesture = None
recognized_face = None

def reset():
    global _stop, __isRunning, x_dis, z_dis, current_gesture, recognized_face
    global motion_detector, depth_estimator, zoom_controller
    x_dis = 1500
    y_dis = 6
    z_dis = 18
    _stop = False
    __isRunning = False
    current_gesture = None
    recognized_face = None
    
    # Reset modular components
    motion_detector.reset()
    depth_estimator.reset()
    zoom_controller.reset()

def init():
    global arm_controller
    print("Gesture/Face Tracking Init")
    load_config()
    
    # Initialize arm controller
    arm_controller = ArmController()
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Gesture/Face Tracking Start")

def stop():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    motion_detector.control_spotlight(False)
    print("Gesture/Face Tracking Stop")

def exit():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    motion_detector.control_spotlight(False)
    print("Gesture/Face Tracking Exit")

def setDetectionMode(mode):
    global detection_mode
    if mode in ['face', 'hand']:
        detection_mode = mode
        print(f"[INFO] Detection mode set to: {mode}")
    else:
        print(f"[ERROR] Invalid detection mode: {mode}")

def set_zoom(level):
    zoom_controller.set_zoom(level)

def apply_zoom(frame, center_x=None, center_y=None):
    return zoom_controller.apply_zoom(frame, center_x, center_y)

def map_coordinates(x, y):
    return zoom_controller.map_coordinates(x, y)

def detect_gesture(hand_landmarks):
    return gesture_detector.detect_gesture(hand_landmarks)

def detect_face(frame):
    return face_detector.detect_face(frame)

def recognize_face_mediapipe(face_img):
    # This functionality is now handled by FaceDetector
    return face_detector._recognize_face_mediapipe(face_img)

def detect_hand(frame):
    return gesture_detector.detect_hand(frame)

def estimate_depth(area, current_z):
    return depth_estimator.estimate_depth(area, current_z)

def get_depth_color(depth):
    return depth_estimator.get_depth_color(depth)

def run(img):
    global __isRunning, x_dis, y_dis, z_dis, current_gesture, recognized_face
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img

    # Apply zoom if enabled
    if zoom_controller.get_current_zoom() > 1.0:
        img_copy = zoom_controller.apply_zoom(img_copy)

    # Detect motion
    motion_center, motion_area = motion_detector.detect_motion(img_copy)
    if motion_center is not None:
        # Motion detected, turn on spotlight
        motion_detector.control_spotlight(True, 100, (255, 255, 255))  # Full brightness white
        # Draw motion detection
        img_copy = motion_detector.draw_motion_detection(img_copy, motion_center, motion_area)
    else:
        # Check if motion has timed out
        motion_detector.check_motion_timeout()

    # Detect based on current mode
    if detection_mode == 'face':
        result = face_detector.detect_face(img_copy)
        if result[0] is not None:
            center, area, name = result
            if zoom_controller.get_current_zoom() > 1.0:
                center = zoom_controller.map_coordinates(center[0], center[1])
            recognized_face = name
            color = (0, 255, 0)  # Green for face
            
            # Draw face detection
            img_copy = face_detector.draw_face_detection(img_copy, center, area, name)
        else:
            center, area = None, 0
            recognized_face = None
            color = (0, 255, 0)
    else:
        result = gesture_detector.detect_hand(img_copy)
        if result[0] is not None:
            center, area, gesture = result
            if zoom_controller.get_current_zoom() > 1.0:
                center = zoom_controller.map_coordinates(center[0], center[1])
            current_gesture = gesture
            color = (255, 0, 0)  # Blue for hand
            
            # Draw hand landmarks
            if gesture:
                # Get hand landmarks for drawing
                import mediapipe as mp
                rgb_frame = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
                results = gesture_detector.hands.process(rgb_frame)
                if results.multi_hand_landmarks:
                    img_copy = gesture_detector.draw_hand_landmarks(img_copy, results.multi_hand_landmarks[0])
                
                cv2.putText(img_copy, f'Gesture: {gesture}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        else:
            center, area = None, 0
            current_gesture = None
            color = (255, 0, 0)

    if center is not None and area > 1000:
        center_x, center_y = center
        
        # Estimate depth
        depth, confidence = depth_estimator.estimate_depth(area, z_dis)
        
        # Get color based on depth
        depth_color = depth_estimator.get_depth_color(depth)
        
        # Draw depth information
        img_copy = depth_estimator.draw_depth_info(img_copy, depth, confidence)

        # Update PID and move arm
        x_pid.SetPoint = center_x
        y_pid.SetPoint = center_y
        x_pid.update(center_x)
        y_pid.update(center_y)
        x_dis += x_pid.output
        y_dis += y_pid.output
        if arm_controller:
            arm_controller.AK.setPitchRangeMoving((x_dis, y_dis, z_dis), 0, -90, 90, 1500)

    # Draw zoom level indicator
    img_copy = zoom_controller.draw_zoom_info(img_copy)
    
    # Draw spotlight status
    spotlight_status = "ON" if motion_detector.is_spotlight_on() else "OFF"
    cv2.putText(img_copy, f'Spotlight: {spotlight_status}', 
                (10, img_h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    return img_copy

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
    
    print("\n=== Gesture/Face Tracking Controls ===")
    print("f - Switch to Face Detection Mode")
    print("h - Switch to Hand Gesture Detection Mode")
    print("z - Cycle through zoom levels")
    print("l - Toggle spotlight")
    print("u - Toggle undistort")
    print("q - Quit")
    print("=====================================\n")
    
    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if undistort_enabled and K is not None and D is not None:
                frame_bgr = lab_auto_calibration.undistort_frame(frame_bgr, K, D)
            Frame = run(frame_bgr)
            frame_resize = cv2.resize(Frame, (640, 480))
            
            # Overlay key info
            cv2.putText(frame_resize, f'Mode: {detection_mode.upper()}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(frame_resize, '[f] Face | [h] Hand | [z] Zoom | [l] Light | [u] Undistort | [q] Quit', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            if recognized_face:
                cv2.putText(frame_resize, f'Recognized: {recognized_face}', (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if current_gesture:
                cv2.putText(frame_resize, f'Gesture: {current_gesture}', (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            cv2.imshow('IMX477 Gesture/Face Tracking', frame_resize)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):  # Switch to face detection
                setDetectionMode('face')
                print("[INFO] Switched to Face Detection Mode")
            elif key == ord('h'):  # Switch to hand detection
                setDetectionMode('hand')
                print("[INFO] Switched to Hand Gesture Detection Mode")
            elif key == ord('z'):  # Cycle through zoom levels
                current_index = zoom_controller.get_zoom_levels().index(zoom_controller.get_current_zoom())
                next_index = (current_index + 1) % len(zoom_controller.get_zoom_levels())
                set_zoom(zoom_controller.get_zoom_levels()[next_index])
            elif key == ord('l'):  # Toggle spotlight
                control_spotlight(not motion_detector.is_spotlight_on())
            elif key == ord('u'):
                undistort_enabled = not undistort_enabled
                print(f"Undistortion {'enabled' if undistort_enabled else 'disabled'}")
            elif key == ord('q') or key == 27:
                print("Quitting...")
                break
        else:
            time.sleep(0.01)
    
    stop()
    cv2.destroyAllWindows() 