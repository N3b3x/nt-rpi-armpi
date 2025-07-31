#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
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
from tracking_components.zoom_controller import ZoomController

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

# Face tracking buffer to prevent immediate loss
face_tracking_buffer = {
    'last_center_x': -1,
    'last_center_y': -1,
    'last_area': 0,
    'buffer_frames': 0,
    'max_buffer_frames': 10  # Keep face data for 10 frames after detection loss
}

# Initialize app
def init():
    global arm_controller, zoom_controller, last_zoom_change_time, zoom_transition_active
    print("Face Tracker Init")
    load_config()
    
    # Initialize arm controller
    arm_controller = ArmController()
    
    # Initialize zoom controller for auto-zoom functionality
    zoom_controller = ZoomController()
    
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
    
    # More responsive tracking parameters
    consecutive_off_center_frames = 0
    required_consecutive_frames = 2  # Reduced from 3 - more responsive
    last_movement_time = 0
    min_movement_interval = 0.15  # Reduced from 0.3 - faster response
    
    print(f"[Track] Entering continuous tracking mode - starting from base position {base_pos}")
    
    while __isRunning and tracking_mode and arm_controller:
        # Get current face position from run() function
        # Skip tracking during zoom transitions to prevent face loss
        if zoom_controller.is_transitioning():
            time.sleep(0.05)
            continue
            
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
            
            # Auto-zoom logic based on face size with improved stability
            current_zoom = zoom_controller.get_current_zoom()
            target_zoom = current_zoom
            
            # Only apply zoom if face is actually detected (area > 0) and not transitioning
            if area > 0 and not zoom_controller.is_transitioning():
                # Much more conservative zoom thresholds to prevent face loss
                # Small faces (far away) - zoom in gradually
                if area < 20000:  # Very small face - zoom in aggressively (further reduced threshold)
                    if current_zoom < 2.0:
                        target_zoom = 2.0
                        print(f"[ZOOM] Face very small ({area} pixels), zooming in to 2.0x")
                elif area < 40000:  # Small face - moderate zoom (further reduced threshold)
                    if current_zoom < 1.5:
                        target_zoom = 1.5
                        print(f"[ZOOM] Face small ({area} pixels), zooming in to 1.5x")
                # Large faces (too close) - zoom out
                elif area > 200000:  # Very large face - zoom out (increased threshold)
                    if current_zoom > 1.0:
                        target_zoom = 1.0
                        print(f"[ZOOM] Face very large ({area} pixels), zooming out to 1.0x")
                elif area > 170000:  # Large face - moderate zoom out (increased threshold)
                    if current_zoom > 1.0:
                        target_zoom = 1.0
                        print(f"[ZOOM] Face large ({area} pixels), zooming out to 1.0x")
                # Good face size - maintain current zoom (very large stable range)
                elif 40000 <= area <= 170000:  # Good size range - no zoom change (expanded range)
                    pass  # Keep current zoom level
            
            # Apply zoom change if needed (zoom controller handles timing and transitions)
            if target_zoom != current_zoom:
                zoom_controller.set_zoom(target_zoom)
            
            # More responsive tracking
            img_w, img_h = 1920, 1080  # Camera resolution
            center_threshold = 200  # Increased from 150 - larger dead zone to prevent jitter
            
            # Calculate how far the face is from center
            face_offset = center_x - img_w/2
            
            # Only move if face is significantly off-center AND has been off-center for a few frames
            # AND enough time has passed since last movement
            current_time = time.time()
            if abs(face_offset) > center_threshold and (current_time - last_movement_time) > min_movement_interval:
                consecutive_off_center_frames += 1
                print(f"[Track] Face off-center at {center_x} (offset {face_offset:.1f}) - frame {consecutive_off_center_frames}/{required_consecutive_frames}")
                
                # Only move after a few consecutive frames of being off-center
                if consecutive_off_center_frames >= required_consecutive_frames:
                    # Calculate larger adjustment for more responsive tracking
                    adjustment = face_offset * 0.03  # Doubled from 0.015 - much more responsive
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
            
            if face_lost_count > 60:  # ~3 seconds at 20Hz - shorter timeout for more responsive recovery
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
    
    # Start scanning from current arm position instead of predefined angles
    current_angle = arm_controller.current_base_angle
    print(f"[Scan] Starting scan from current arm position: {current_angle}°")
    
    # Define scanning angles (cover ±90° range - more conservative)
    angles = generate_polar_scan_angles(-90, 90, 60)  # 60° intervals for face scanning
    
    # Find the closest angle in our predefined list to start from
    angle_idx = 0
    min_distance = float('inf')
    for i, angle in enumerate(angles):
        distance = abs(angle - current_angle)
        if distance < min_distance:
            min_distance = distance
            angle_idx = i
    
    print(f"[Scan] Closest predefined angle to current position ({current_angle}°) is {angles[angle_idx]}° at index {angle_idx}")
    print(f"[Scan] Generated {len(angles)} scan angles: {angles}")
    
    scan_direction = 1  # 1 for forward, -1 for reverse
    fixed_pitch = 0  # Upright pitch angle for face-level scanning
    
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

            # Only rotate if we're not already at the target angle (within 5° tolerance)
            angle_diff = abs(base_angle - arm_controller.current_base_angle)
            if angle_diff > 5:
                rotation_needed = base_angle - arm_controller.current_base_angle
                arm_controller.rotate_base(rotation_needed)
                time.sleep(0.15)  # Stabilization time
            else:
                print(f"[Scan] Already at target angle {base_angle}° (current: {arm_controller.current_base_angle}°)")

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
    
    # SMART DETECTION APPROACH: Use original frame for detection (no zoom)
    # This eliminates shakiness while maintaining detection quality
    # Zoom will be applied only to display frame later
    
    # Process face detection on the original frame (no zoom)
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    results = faceDetection.process(imgRGB)  # Process each frame with face detection

    if results.detections:  # If faces are detected
        if tracking_mode:
            print(f"[Run] Face detection: {len(results.detections)} faces detected with scores: {[d.score[0] for d in results.detections]}")

        for index, detection in enumerate(results.detections):  # Return face index and keypoint coordinates
            scores = list(detection.score)
            # Much more sensitive confidence threshold for longer range detection
            confidence_threshold = 0.1 if tracking_mode else 0.25  # Even lower threshold for tracking mode
            if scores and scores[0] > confidence_threshold:
                bboxC = detection.location_data.relative_bounding_box  # Set bounding box for all boxes' xywh and keypoint info
                
                # Convert bounding box coordinates from relative to pixel coordinates
                bbox = (
                    int(bboxC.xmin * img_w),
                    int(bboxC.ymin * img_h),
                    int(bboxC.width * img_w),
                    int(bboxC.height * img_h)
                )
                
                cv2.rectangle(img_copy, bbox, (0, 255, 0), 2)  # Draw rectangle on original frame
                
                # Get recognition box info, xy is upper left corner coordinates
                x, y, w, h = bbox
                center_x = int(x + (w / 2))
                center_y = int(y + (h / 2))
                area = int(w * h)
                
                # Update face tracking buffer with current face data
                face_tracking_buffer['last_center_x'] = center_x
                face_tracking_buffer['last_center_y'] = center_y
                face_tracking_buffer['last_area'] = area
                face_tracking_buffer['buffer_frames'] = 0
                
                if action_finish:
                    start_greet = True

    else:
        if tracking_mode:
            print(f"[Run] No faces detected in frame")
            # Use buffer data if available to prevent immediate face loss
            if face_tracking_buffer['buffer_frames'] < face_tracking_buffer['max_buffer_frames']:
                center_x = face_tracking_buffer['last_center_x']
                center_y = face_tracking_buffer['last_center_y']
                area = face_tracking_buffer['last_area']
                face_tracking_buffer['buffer_frames'] += 1
                print(f"[Run] Using buffered face data (frame {face_tracking_buffer['buffer_frames']}/{face_tracking_buffer['max_buffer_frames']})")
            else:
                center_x, center_y, area = -1, -1, 0
        else:
            center_x, center_y, area = -1, -1, 0
            
    return img_copy  # Return the original frame (no zoom applied to detection)

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
                
                # Apply zoom ONLY to display frame (not detection frame)
                # This gives user zoom view without affecting detection stability
                if tracking_mode and zoom_controller.get_current_zoom() > 1.0:
                    # Use detected face center for zoom, or frame center if no face
                    # Ensure we have valid face coordinates before applying zoom
                    if center_x != -1 and center_y != -1 and area > 100:
                        # Don't zoom if face is too small (could cause face loss)
                        if area > 5000:  # Minimum face area for safe zooming
                            zoom_center_x = center_x
                            zoom_center_y = center_y
                            print(f"[ZOOM] Applying zoom to face at ({zoom_center_x}, {zoom_center_y}), area: {area}")
                        else:
                            # Face too small, don't zoom
                            print(f"[ZOOM] Face too small (area: {area}), not zooming")
                            zoom_center_x = 1920 // 2
                            zoom_center_y = 1080 // 2
                    else:
                        # If no face detected, use frame center
                        zoom_center_x = 1920 // 2
                        zoom_center_y = 1080 // 2
                        print(f"[ZOOM] No face detected, using frame center ({zoom_center_x}, {zoom_center_y})")
                    
                    Frame = zoom_controller.apply_zoom(Frame, zoom_center_x, zoom_center_y)
                
                # Keep display at full resolution for better zoom visibility
                # No resize - show full 1920x1080 resolution
                frame_resize = Frame
                
                # Show tracking mode status
                mode_text = "TRACKING" if tracking_mode else "SCANNING"
                mode_color = (0, 255, 0) if tracking_mode else (255, 255, 0)
                cv2.putText(frame_resize, f'Mode: {mode_text}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
                
                # Show zoom information
                zoom_level = zoom_controller.get_current_zoom()
                if zoom_level > 1.0:
                    zoom_text = f'Zoom: {zoom_level:.1f}x'
                    if zoom_controller.is_transitioning():
                        zoom_text += f' → {zoom_controller.get_target_zoom():.1f}x'
                    cv2.putText(frame_resize, zoom_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                # Overlay key info
                cv2.putText(frame_resize, '[SPACE] Start scan', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame_resize, '[u] Toggle undistort', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame_resize, '[q] Quit', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                
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