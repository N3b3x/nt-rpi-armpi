#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import mediapipe as mp
from Camera import Camera
from hiwonder import FaceDetect
from hiwonder import FaceRecognition

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

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

# Camera settings
CAMERA_RESOLUTION = (1920, 1080)  # Higher resolution for better long-range detection
DISPLAY_RESOLUTION = (640, 480)   # Lower resolution for display
ZOOM_LEVELS = [1.0, 1.5, 2.0, 2.5, 3.0]  # Available zoom levels
current_zoom = 1.0
zoom_region = None  # (x, y, w, h) for zoomed region

# Motion detection settings
MOTION_THRESHOLD = 25  # Minimum pixel difference to trigger motion
MOTION_AREA_THRESHOLD = 500  # Minimum area of motion to trigger spotlight
MOTION_HISTORY = 5  # Number of frames to keep for motion detection
motion_frames = []  # List to store previous frames
last_motion_time = 0
MOTION_TIMEOUT = 5  # Seconds to keep spotlight on after last motion

# Spotlight control
spotlight_on = False
spotlight_brightness = 100  # 0-100
spotlight_color = (255, 255, 255)  # RGB color

# Initialize MediaPipe for hand tracking
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize MediaPipe Hands with higher confidence for long-range
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,  # Increased confidence threshold
    min_tracking_confidence=0.7
)

# Initialize HiWonder face detection and recognition
face_detect = FaceDetect()
face_recognition = FaceRecognition()

# Define gestures
GESTURES = {
    'POINTING': 'Pointing',
    'GRAB': 'Grab',
    'OPEN': 'Open Hand',
    'FIST': 'Fist',
    'PEACE': 'Peace',
    'THUMB_UP': 'Thumb Up'
}

# Depth estimation settings
DEPTH_HISTORY = 10  # Number of frames to keep for depth estimation
depth_history = []  # List to store previous depth measurements
REFERENCE_DISTANCE = 30  # Reference distance in cm
REFERENCE_SIZE = 1000  # Reference size at REFERENCE_DISTANCE
current_depth = None
depth_confidence = 0.0

def initMove():
    board.pwm_servo_set_position(0.8, [[1, servo1]])
    AK.setPitchRangeMoving((0, y_dis, z_dis), 0,-90, 90, 1500)

def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

def control_spotlight(on=True, brightness=100, color=(255, 255, 255)):
    """Control the spotlight mounted on the arm"""
    global spotlight_on, spotlight_brightness, spotlight_color
    spotlight_on = on
    spotlight_brightness = brightness
    spotlight_color = color
    
    if on:
        # Convert RGB to PWM values (assuming 0-100 range)
        r = int(color[0] * brightness / 100)
        g = int(color[1] * brightness / 100)
        b = int(color[2] * brightness / 100)
        # Set RGB values for the spotlight
        board.set_rgb([[1, r, g, b], [2, r, g, b]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

def detect_motion(frame):
    """Detect motion in the frame"""
    global motion_frames, last_motion_time
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
    # Add current frame to history
    motion_frames.append(gray)
    if len(motion_frames) > MOTION_HISTORY:
        motion_frames.pop(0)
    
    # Need at least 2 frames to detect motion
    if len(motion_frames) < 2:
        return None, 0
    
    # Calculate difference between current and previous frame
    frame_delta = cv2.absdiff(motion_frames[-2], motion_frames[-1])
    thresh = cv2.threshold(frame_delta, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)[1]
    
    # Dilate the thresholded image to fill in holes
    thresh = cv2.dilate(thresh, None, iterations=2)
    
    # Find contours of motion
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area > MOTION_AREA_THRESHOLD:
            # Get the bounding box of the motion
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w//2
            center_y = y + h//2
            last_motion_time = time.time()
            return (center_x, center_y), area
    
    return None, 0

def check_motion_timeout():
    """Check if motion has timed out and turn off spotlight if needed"""
    global spotlight_on, last_motion_time
    if spotlight_on and time.time() - last_motion_time > MOTION_TIMEOUT:
        control_spotlight(False)

_stop = False
__isRunning = False
detection_mode = 'face'  # 'face' or 'hand'
current_gesture = None
recognized_face = None

def reset():
    global _stop, __isRunning, x_dis, z_dis, current_gesture, recognized_face, current_zoom, zoom_region
    global motion_frames, last_motion_time, spotlight_on, depth_history, current_depth, depth_confidence
    x_dis = 1500
    y_dis = 6
    z_dis = 18
    _stop = False
    __isRunning = False
    current_gesture = None
    recognized_face = None
    current_zoom = 1.0
    zoom_region = None
    motion_frames = []
    last_motion_time = 0
    spotlight_on = False
    control_spotlight(False)
    depth_history = []
    current_depth = None
    depth_confidence = 0.0

def init():
    print("Gesture/Face Tracking Init")
    initMove()
    # Initialize face recognition database
    face_recognition.load_known_faces()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Gesture/Face Tracking Start")

def stop():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    control_spotlight(False)
    print("Gesture/Face Tracking Stop")

def exit():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    control_spotlight(False)
    print("Gesture/Face Tracking Exit")

def setDetectionMode(mode):
    global detection_mode
    if mode in ['face', 'hand']:
        detection_mode = mode
        return (True, ())
    return (False, ())

def set_zoom(level):
    global current_zoom, zoom_region
    if level in ZOOM_LEVELS:
        current_zoom = level
        zoom_region = None  # Reset zoom region when changing zoom level
        return True
    return False

def apply_zoom(frame, center_x=None, center_y=None):
    global zoom_region
    if current_zoom == 1.0:
        return frame
    
    h, w = frame.shape[:2]
    
    # If no center point provided, use frame center
    if center_x is None or center_y is None:
        center_x, center_y = w // 2, h // 2
    
    # Calculate zoom region
    zoom_w = int(w / current_zoom)
    zoom_h = int(h / current_zoom)
    
    # Ensure zoom region stays within frame bounds
    x1 = max(0, center_x - zoom_w // 2)
    y1 = max(0, center_y - zoom_h // 2)
    x2 = min(w, x1 + zoom_w)
    y2 = min(h, y1 + zoom_h)
    
    # Adjust if region exceeds bounds
    if x2 > w:
        x1 = w - zoom_w
    if y2 > h:
        y1 = h - zoom_h
    
    # Extract and resize zoomed region
    zoomed = frame[y1:y2, x1:x2]
    zoomed = cv2.resize(zoomed, (w, h))
    
    # Store zoom region for coordinate mapping
    zoom_region = (x1, y1, x2 - x1, y2 - y1)
    
    return zoomed

def map_coordinates(x, y):
    """Map coordinates from zoomed frame back to original frame"""
    if zoom_region is None:
        return x, y
    
    x1, y1, w, h = zoom_region
    return int(x1 + x * w / CAMERA_RESOLUTION[0]), int(y1 + y * h / CAMERA_RESOLUTION[1])

def detect_gesture(hand_landmarks):
    """Detect hand gesture based on finger positions"""
    if not hand_landmarks:
        return None
    
    # Get finger tip and pip landmarks
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    
    # Get finger pip landmarks for bend detection
    index_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    middle_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    ring_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
    pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]
    
    # Check for pointing gesture (only index finger extended)
    if (index_tip.y < index_pip.y and 
        middle_tip.y > middle_pip.y and 
        ring_tip.y > ring_pip.y and 
        pinky_tip.y > pinky_pip.y):
        return GESTURES['POINTING']
    
    # Check for grab gesture (all fingers bent)
    if (index_tip.y > index_pip.y and 
        middle_tip.y > middle_pip.y and 
        ring_tip.y > ring_pip.y and 
        pinky_tip.y > pinky_pip.y):
        return GESTURES['GRAB']
    
    # Check for open hand (all fingers extended)
    if (index_tip.y < index_pip.y and 
        middle_tip.y < middle_pip.y and 
        ring_tip.y < ring_pip.y and 
        pinky_tip.y < pinky_pip.y):
        return GESTURES['OPEN']
    
    # Check for peace sign (index and middle fingers extended)
    if (index_tip.y < index_pip.y and 
        middle_tip.y < middle_pip.y and 
        ring_tip.y > ring_pip.y and 
        pinky_tip.y > pinky_pip.y):
        return GESTURES['PEACE']
    
    # Check for thumb up
    if (thumb_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].y and
        index_tip.y > index_pip.y and 
        middle_tip.y > middle_pip.y and 
        ring_tip.y > ring_pip.y and 
        pinky_tip.y > pinky_pip.y):
        return GESTURES['THUMB_UP']
    
    return None

def detect_face(frame):
    # Use HiWonder's face detection
    faces = face_detect.detect(frame)
    if faces:
        # Get the largest face
        largest_face = max(faces, key=lambda x: x['w'] * x['h'])
        x, y, w, h = largest_face['x'], largest_face['y'], largest_face['w'], largest_face['h']
        center_x = x + w//2
        center_y = y + h//2
        
        # Try to recognize the face
        face_img = frame[y:y+h, x:x+w]
        name = face_recognition.recognize(face_img)
        
        return (center_x, center_y), w * h, name
    return None, 0, None

def detect_hand(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)
    
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        # Get hand center
        wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
        h, w = frame.shape[:2]
        center_x = int(wrist.x * w)
        center_y = int(wrist.y * h)
        
        # Detect gesture
        gesture = detect_gesture(hand_landmarks)
        return (center_x, center_y), 1000, gesture  # Arbitrary area value
    return None, 0, None

def estimate_depth(area, current_z):
    """Estimate depth using object size and arm position"""
    global depth_history, current_depth, depth_confidence
    
    if area <= 0:
        return None, 0.0
    
    # Calculate depth using inverse square law
    # depth = reference_distance * sqrt(reference_size / current_size)
    estimated_depth = REFERENCE_DISTANCE * math.sqrt(REFERENCE_SIZE / area)
    
    # Adjust for current arm position
    adjusted_depth = estimated_depth + (current_z - Z_DIS)
    
    # Add to history
    depth_history.append(adjusted_depth)
    if len(depth_history) > DEPTH_HISTORY:
        depth_history.pop(0)
    
    # Calculate average and confidence
    if len(depth_history) > 0:
        avg_depth = sum(depth_history) / len(depth_history)
        # Calculate confidence based on variance
        variance = sum((x - avg_depth) ** 2 for x in depth_history) / len(depth_history)
        confidence = 1.0 / (1.0 + variance)  # Higher variance = lower confidence
        
        current_depth = avg_depth
        depth_confidence = confidence
        return avg_depth, confidence
    
    return None, 0.0

def get_depth_color(depth):
    """Get color based on depth (red = close, green = medium, blue = far)"""
    if depth is None:
        return (255, 255, 255)  # White for unknown
    
    # Define depth ranges
    CLOSE_THRESHOLD = 50  # cm
    FAR_THRESHOLD = 150   # cm
    
    if depth < CLOSE_THRESHOLD:
        # Red to yellow gradient
        ratio = depth / CLOSE_THRESHOLD
        return (255, int(255 * ratio), 0)
    elif depth < FAR_THRESHOLD:
        # Yellow to green gradient
        ratio = (depth - CLOSE_THRESHOLD) / (FAR_THRESHOLD - CLOSE_THRESHOLD)
        return (int(255 * (1 - ratio)), 255, 0)
    else:
        # Green to blue gradient
        ratio = min(1.0, (depth - FAR_THRESHOLD) / FAR_THRESHOLD)
        return (0, int(255 * (1 - ratio)), int(255 * ratio))

def run(img):
    global __isRunning, x_dis, y_dis, z_dis, current_gesture, recognized_face, current_depth, depth_confidence
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img

    # Apply zoom if enabled
    if current_zoom > 1.0:
        img_copy = apply_zoom(img_copy)

    # Detect motion
    motion_center, motion_area = detect_motion(img_copy)
    if motion_center is not None:
        # Motion detected, turn on spotlight
        control_spotlight(True, 100, (255, 255, 255))  # Full brightness white
        # Draw motion detection
        cv2.circle(img_copy, motion_center, 5, (0, 255, 0), -1)
        cv2.putText(img_copy, 'Motion Detected', 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    else:
        # Check if motion has timed out
        check_motion_timeout()

    # Detect based on current mode
    if detection_mode == 'face':
        result = detect_face(img_copy)
        if result[0] is not None:
            center, area, name = result
            if current_zoom > 1.0:
                center = map_coordinates(center[0], center[1])
            recognized_face = name
            color = (0, 255, 0)  # Green for face
        else:
            center, area = None, 0
            recognized_face = None
            color = (0, 255, 0)
    else:
        result = detect_hand(img_copy)
        if result[0] is not None:
            center, area, gesture = result
            if current_zoom > 1.0:
                center = map_coordinates(center[0], center[1])
            current_gesture = gesture
            color = (255, 0, 0)  # Blue for hand
        else:
            center, area = None, 0
            current_gesture = None
            color = (255, 0, 0)

    if center is not None and area > 1000:
        center_x, center_y = center
        
        # Estimate depth
        depth, confidence = estimate_depth(area, z_dis)
        
        # Get color based on depth
        depth_color = get_depth_color(depth)
        
        # Draw detection
        if detection_mode == 'face':
            # Draw face rectangle and name
            x, y, w, h = center_x - area//2, center_y - area//2, int(math.sqrt(area)), int(math.sqrt(area))
            cv2.rectangle(img_copy, (x, y), (x + w, y + h), depth_color, 2)
            if recognized_face:
                cv2.putText(img_copy, f'Face: {recognized_face}', 
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, depth_color, 2)
        else:
            # Draw hand landmarks and gesture
            mp_drawing.draw_landmarks(
                image=img_copy,
                landmark_list=results.multi_hand_landmarks[0],
                connections=mp_hands.HAND_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=depth_color, thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=depth_color, thickness=2)
            )
            if current_gesture:
                cv2.putText(img_copy, f'Gesture: {current_gesture}', 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, depth_color, 2)

        # Update PID and move arm
        x_pid.SetPoint = center_x
        y_pid.SetPoint = center_y
        x_pid.update(center_x)
        y_pid.update(center_y)
        x_dis += x_pid.output
        y_dis += y_pid.output
        AK.setPitchRangeMoving((x_dis, y_dis, z_dis), 0, -90, 90, 1500)

    # Draw zoom level indicator
    cv2.putText(img_copy, f'Zoom: {current_zoom}x', 
                (10, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    
    # Draw spotlight status
    spotlight_status = "ON" if spotlight_on else "OFF"
    cv2.putText(img_copy, f'Spotlight: {spotlight_status}', 
                (10, img_h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    
    # Draw depth information
    if current_depth is not None:
        depth_text = f'Depth: {current_depth:.1f}cm (Conf: {depth_confidence:.2f})'
        cv2.putText(img_copy, depth_text, 
                    (10, img_h - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    return img_copy

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    camera = Camera(resolution=CAMERA_RESOLUTION)
    
    while True:
        frame = camera.get_frame()
        if frame is not None:
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, DISPLAY_RESOLUTION)
            cv2.imshow('IMX477 Gesture/Face Tracking', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
            elif key == ord('f'):  # Switch to face detection
                setDetectionMode('face')
            elif key == ord('h'):  # Switch to hand detection
                setDetectionMode('hand')
            elif key == ord('z'):  # Cycle through zoom levels
                current_index = ZOOM_LEVELS.index(current_zoom)
                next_index = (current_index + 1) % len(ZOOM_LEVELS)
                set_zoom(ZOOM_LEVELS[next_index])
            elif key == ord('l'):  # Toggle spotlight
                control_spotlight(not spotlight_on)
        else:
            time.sleep(0.01)
    
    stop()
    cv2.destroyAllWindows() 