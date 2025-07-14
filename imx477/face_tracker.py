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

# Customize face detection method, lower confidence for longer range detection
faceDetection = Face.FaceDetection(min_detection_confidence=0.3)

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

# Initialize arm controller
arm_controller = None

# Initial position for face scanning
def initMove():
    global arm_controller
    if arm_controller:
        arm_controller.init_move_face_scan()
    
start_greet = False
action_finish = True

# Reset variables
def reset():
    global start_greet
    global action_finish
    global tracking_mode
    global face_lost_count
    global center_x, center_y, area

    start_greet = False
    action_finish = True
    tracking_mode = False  # False = scanning, True = continuous tracking
    face_lost_count = 0
    center_x, center_y, area = -1, -1, 0
    initMove()  

__isRunning = False
tracking_mode = False  # False = scanning, True = continuous tracking
face_lost_count = 0
face_detected_time = 0
center_x, center_y, area = -1, -1, 0
scan_started = False  # Flag to track when SPACE is pressed

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
    print("DEBUG: Camera initialized, waiting for user to start scan...")
    # Don't start scan thread yet - wait for user input

def continuous_tracking():
    """
    Continuous tracking mode - once face is detected, continuously track it.
    Very conservative approach to prevent drift.
    """
    global tracking_mode, face_lost_count, face_detected_time
    global center_x, center_y, area
    
    # Simple tracking - start from current arm position
    base_pos = arm_controller.current_base_pos if arm_controller else 1500
    last_face_time = time.time()
    
    # Conservative hysteresis to prevent drift
    consecutive_off_center_frames = 0
    required_consecutive_frames = 3  # Must be off-center for 3 frames before moving (reduced from 5)
    last_movement_time = 0
    min_movement_interval = 0.3  # Minimum 0.3 seconds between movements (reduced from 0.5)
    
    print(f"[Track] Entering continuous tracking mode - starting from base position {base_pos}")
    
    while __isRunning and tracking_mode and arm_controller:
        # Get current face position from run() function
        if center_x != -1 and center_y != -1 and area > 100:  # Reduced minimum area for longer range detection
            # Face detected - track it
            face_lost_count = 0
            face_detected_time = time.time()
            last_face_time = time.time()
            
            # Debug: Check variable types
            print(f"[Track] Debug - center_x: {center_x} ({type(center_x)}), center_y: {center_y} ({type(center_y)}), area: {area} ({type(area)})")
            
            # Ensure variables are numbers
            if not isinstance(center_x, (int, float)) or not isinstance(center_y, (int, float)) or not isinstance(area, (int, float)):
                print(f"[Track] Error: Invalid variable types - center_x: {type(center_x)}, center_y: {type(center_y)}, area: {type(area)}")
                continue
            
            # Conservative tracking
            img_w, img_h = 1920, 1080  # Camera resolution
            center_threshold = 150  # pixels from center - smaller dead zone for more responsive tracking
            
            # Calculate how far the face is from center
            face_offset = center_x - img_w/2
            
            # Only move if face is significantly off-center AND has been off-center for many frames
            # AND enough time has passed since last movement
            current_time = time.time()
            if abs(face_offset) > center_threshold and (current_time - last_movement_time) > min_movement_interval:
                consecutive_off_center_frames += 1
                print(f"[Track] Face off-center at {center_x} (offset {face_offset:.1f}) - frame {consecutive_off_center_frames}/{required_consecutive_frames}")
                
                # Only move after many consecutive frames of being off-center
                if consecutive_off_center_frames >= required_consecutive_frames:
                    # Calculate small adjustment - conservative but responsive
                    adjustment = face_offset * 0.015  # Slightly larger adjustment factor for more responsive tracking
                    new_base_pos = base_pos + adjustment
                    
                    # Debug direction change
                    direction = "RIGHT" if face_offset > 0 else "LEFT"
                    print(f"[Track] Face moving {direction} (offset {face_offset:.1f}), adjusting base from {base_pos:.1f} to {new_base_pos:.1f}")
                    
                    # Only move if the new position is within safe limits
                    if 800 <= new_base_pos <= 2200:  # Conservative range
                        base_pos = new_base_pos
                        # Move base servo using the same method as scanning (IK system)
                        try:
                            # Use the same method as the IK system that works during scanning
                            deviation = arm_controller.deviation_data.get('6', 0)
                            final_pos = int(base_pos) + deviation
                            # Use the same format as the IK system: board.pwm_servo_set_position(float(movetime)/1000.0, [[6, int(servos[3]+deviation_data['6'])]])
                            arm_controller.board.pwm_servo_set_position(0.1, [[6, final_pos]])
                            arm_controller.current_base_pos = int(base_pos)  # Update the tracked position
                            print(f"[Track] Following face: moved base to {base_pos:.1f} (face at {center_x}, offset {face_offset:.1f})")
                            last_movement_time = current_time
                        except Exception as e:
                            print(f"[Track] Error moving base: {e}")
                    else:
                        print(f"[Track] Face at {center_x} (offset {face_offset:.1f}) but base position {new_base_pos:.1f} out of safe range")
                    
                    # Reset counter after moving
                    consecutive_off_center_frames = 0
            else:
                # Face is centered or not enough time passed, reset counter
                consecutive_off_center_frames = 0
                if abs(face_offset) <= center_threshold:
                    print(f"[Track] Face centered at {center_x}, no movement needed")
                else:
                    print(f"[Track] Face off-center but waiting for time interval ({(current_time - last_movement_time):.1f}s)")
            
            # Set RGB to green to indicate tracking
            arm_controller.set_rgb("green")
            
        else:
            # Face lost - increment counter
            face_lost_count += 1
            print(f"[Track] Face lost, count: {face_lost_count} (center_x={center_x}, center_y={center_y}, area={area})")
            
            if face_lost_count > 120:  # ~6 seconds at 20Hz - longer timeout for longer range
                print("[Track] Face lost for too long, returning to scan mode")
                tracking_mode = False
                arm_controller.set_rgb("off")
                break
        
        time.sleep(0.05)  # 20Hz tracking rate

def move():
    """
    Hybrid approach: Polar scanning + Continuous tracking.
    Switches between scanning and tracking modes.
    """
    global start_greet, action_finish, arm_controller, tracking_mode
    
    print("[Scan] Move function started - entering scan loop")
    
    # Define scanning angles (cover ±90° range - more conservative)
    angles = generate_polar_scan_angles(-90, 90, 60)  # 60° intervals for face scanning
    angle_idx = 0
    scan_direction = 1  # 1 for forward, -1 for reverse
    fixed_pitch = 0  # Upright pitch angle for face-level scanning
    
    print(f"[Scan] Generated {len(angles)} scan angles: {angles}")
    
    while __isRunning and arm_controller:
        print(f"[Scan] Loop iteration - tracking_mode: {tracking_mode}, start_greet: {start_greet}, action_finish: {action_finish}")
        if tracking_mode:
            # Switch to continuous tracking mode
            continuous_tracking()
            # After tracking ends, resume scanning
            print("[Scan] Resuming scan mode")
            continue
            
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

                # Check for face detection - but only if we're not already in tracking mode
                if not tracking_mode:
                    for _ in range(3):  # Check multiple times at each position
                        time.sleep(0.05)
                        if start_greet:
                            print(f"[Scan] Face detected at position {pos}")
                            break

                if start_greet:
                    # Face detected - perform greeting action
                    action_finish = False
                    
                    # Longer buzzer beep to indicate face detection
                    arm_controller.board.set_buzzer(1900, 0.5, 0.1, 3)  # 1900Hz beep for 0.5s, repeat 3 times
                    
                    # RGB effect - flash green to indicate face detection
                    arm_controller.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])  # Green
                    time.sleep(0.3)
                    arm_controller.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])  # Off
                    time.sleep(0.2)
                    arm_controller.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])  # Green again
                    time.sleep(0.3)
                    arm_controller.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])  # Off
                    
                    # Use arm controller for gripper movement
                    arm_controller.open_gripper()
                    time.sleep(0.4)
                    arm_controller.close_gripper()
                    
                    # Switch to continuous tracking mode
                    tracking_mode = True
                    print("[Scan] Switching to continuous tracking mode")
                    
                    # Reset variables for tracking mode
                    start_greet = False
                    action_finish = True
                    time.sleep(0.5)
                    break

            # Move to next angle if no face detected
            if not start_greet:
                angle_idx += scan_direction
                
                # Check if we've reached the end of the angle list
                if angle_idx >= len(angles):
                    # Reverse direction and start from the end
                    scan_direction = -1
                    angle_idx = len(angles) - 2  # Start from second-to-last
                    print("[Scan] Reached end of sweep. Reversing direction.")
                elif angle_idx < 0:
                    # Reverse direction and start from the beginning
                    scan_direction = 1
                    angle_idx = 1  # Start from second angle
                    print("[Scan] Reached start of sweep. Reversing direction.")
                else:
                    print(f"[Scan] Moving to next angle: {angles[angle_idx]}°")
        else:
            print(f"[Scan] Waiting - start_greet: {start_greet}, action_finish: {action_finish}")
            # Add timeout to prevent infinite waiting
            if start_greet and action_finish:
                print("[Scan] Timeout - resetting start_greet to continue scanning")
                start_greet = False
            time.sleep(0.01)

size = (640, 480)  # Updated for IMX477 resolution

def run(img):
    global __isRunning, area
    global center_x, center_y
    global start_greet
    global action_finish
    global tracking_mode
    
    if not __isRunning:   # Check if game is started, if not return original image
        return img
    
    # Don't do face detection during extreme arm movements
    if hasattr(arm_controller, 'current_base_pos') and arm_controller.current_base_pos < 800:
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
     
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    results = faceDetection.process(imgRGB)  # Process each frame with face detection

    if results.detections:  # If faces are detected
        if tracking_mode:
            print(f"[Run] Face detection: {len(results.detections)} faces detected with scores: {[d.score[0] for d in results.detections]}")

        for index, detection in enumerate(results.detections):  # Return face index and keypoint coordinates
            scores = list(detection.score)
            # Lower confidence threshold for longer range detection
            confidence_threshold = 0.3 if tracking_mode else 0.5
            if scores and scores[0] > confidence_threshold:
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
        if tracking_mode:
            print(f"[Run] No faces detected in frame")
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
    print("DEBUG: After init() - arm controller initialized")
    start()
    print("DEBUG: After start() - camera ready")
    
    # Start the display loop immediately to open camera window
    print("DEBUG: Starting display loop to open camera window...")
    
    # Create a separate thread for the display loop
    def display_loop():
        global __isRunning, undistort_enabled, K, D, tracking_mode, scan_started
        while __isRunning:
            frame = picam2.capture_array()
            if frame is not None:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if undistort_enabled and K is not None and D is not None:
                    frame_bgr = lab_auto_calibration.undistort_frame(frame_bgr, K, D)
                Frame = run(frame_bgr)
                frame_resize = cv2.resize(Frame, (640, 480))
                
                # Show tracking mode status
                mode_text = "TRACKING" if tracking_mode else "SCANNING"
                mode_color = (0, 255, 0) if tracking_mode else (255, 255, 0)
                cv2.putText(frame_resize, f'Mode: {mode_text}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
                
                # Overlay key info
                cv2.putText(frame_resize, '[SPACE] Start scan', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame_resize, '[u] Toggle undistort', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame_resize, '[q] Quit', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                
                cv2.imshow('IMX477 Face Tracker', frame_resize)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):  # SPACE key
                    scan_started = True
                    print("SPACE pressed - starting scan")
                elif key == ord('u'):
                    undistort_enabled = not undistort_enabled
                    print(f"Undistortion {'enabled' if undistort_enabled else 'disabled'}")
                elif key == ord('q') or key == 27:
                    print("Quitting...")
                    __isRunning = False
                    break
            else:
                time.sleep(0.01)
        cv2.destroyAllWindows()
    
    # Start display loop in separate thread
    display_thread = threading.Thread(target=display_loop, daemon=True)
    display_thread.start()
    
    # Wait a moment for window to open
    time.sleep(2)
    
    # Add user prompt before starting scan
    print("\n" + "="*50)
    print("Face Tracker Ready!")
    print("Camera window should be open now.")
    print("Press SPACE in camera window to start face scanning...")
    print("="*50)
    
    # Wait for SPACE key in camera window instead of terminal input
    scan_started = False
    while not scan_started and __isRunning:
        time.sleep(0.1)
    
    print("DEBUG: SPACE pressed - starting scan thread")
    
    # Start scan thread AFTER user presses SPACE
    print("DEBUG: Starting face scan thread...")
    
    # Reset scan variables before starting
    start_greet = False
    action_finish = True
    print("DEBUG: Reset scan variables - start_greet: False, action_finish: True")
    
    scan_thread = threading.Thread(target=move, args=(), daemon=True)
    scan_thread.start()
    print("DEBUG: Scan thread started successfully")
    print("DEBUG: Scan thread is alive:", scan_thread.is_alive())
    
    # Keep main thread alive
    try:
        while __isRunning:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user")
        __isRunning = False
    
    # Before exiting, reset the camera to clear any stuck color state
    #reset_camera(picam2)
    time.sleep(0.2) 