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

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Initialize MediaPipe Hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Initialize MediaPipe Face Mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Define gestures
GESTURES = {
    'POINTING': 'Pointing',
    'GRAB': 'Grab',
    'OPEN': 'Open Hand',
    'FIST': 'Fist',
    'PEACE': 'Peace',
    'THUMB_UP': 'Thumb Up'
}

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

_stop = False
__isRunning = False
detection_mode = 'face'  # 'face' or 'hand'
current_gesture = None

def reset():
    global _stop, __isRunning, x_dis, z_dis, current_gesture
    x_dis = 1500
    y_dis = 6
    z_dis = 18
    _stop = False
    __isRunning = False
    current_gesture = None

def init():
    print("Gesture/Face Tracking Init")
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
    set_rgb('None')
    print("Gesture/Face Tracking Stop")

def exit():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("Gesture/Face Tracking Exit")

def setDetectionMode(mode):
    global detection_mode
    if mode in ['face', 'hand']:
        detection_mode = mode
        return (True, ())
    return (False, ())

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
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(frame_rgb)
    
    if results.multi_face_landmarks:
        face_landmarks = results.multi_face_landmarks[0]
        # Get face center
        nose_tip = face_landmarks.landmark[1]  # Nose tip landmark
        h, w = frame.shape[:2]
        center_x = int(nose_tip.x * w)
        center_y = int(nose_tip.y * h)
        return (center_x, center_y), 1000  # Arbitrary area value
    return None, 0

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

def run(img):
    global __isRunning, x_dis, y_dis, z_dis, current_gesture
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img

    # Detect based on current mode
    if detection_mode == 'face':
        center, area = detect_face(img_copy)
        color = (0, 255, 0)  # Green for face
        gesture = None
    else:
        result = detect_hand(img_copy)
        if result[0] is not None:
            center, area, gesture = result
            current_gesture = gesture
        else:
            center, area = None, 0
            gesture = None
        color = (255, 0, 0)  # Blue for hand

    if center is not None and area > 1000:
        center_x, center_y = center
        # Draw detection
        if detection_mode == 'face':
            mp_drawing.draw_landmarks(
                image=img_copy,
                landmark_list=results.multi_face_landmarks[0],
                connections=mp_face_mesh.FACEMESH_TESSELATION,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_tesselation_style()
            )
        else:
            mp_drawing.draw_landmarks(
                image=img_copy,
                landmark_list=results.multi_hand_landmarks[0],
                connections=mp_hands.HAND_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=color, thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=color, thickness=2)
            )
            
        # Draw gesture text if detected
        if gesture:
            cv2.putText(img_copy, f'Gesture: {gesture}', 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        # Update PID and move arm
        x_pid.SetPoint = center_x
        y_pid.SetPoint = center_y
        x_pid.update(center_x)
        y_pid.update(center_y)
        x_dis += x_pid.output
        y_dis += y_pid.output
        AK.setPitchRangeMoving((x_dis, y_dis, z_dis), 0, -90, 90, 1500)

    return img_copy

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    camera = Camera(resolution=(640, 480))
    
    while True:
        frame = camera.get_frame()
        if frame is not None:
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('IMX477 Gesture/Face Tracking', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
            elif key == ord('f'):  # Switch to face detection
                setDetectionMode('face')
            elif key == ord('h'):  # Switch to hand detection
                setDetectionMode('hand')
        else:
            time.sleep(0.01)
    
    stop()
    cv2.destroyAllWindows() 