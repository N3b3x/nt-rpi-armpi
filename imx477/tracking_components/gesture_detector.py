#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import mediapipe as mp
import math

class GestureDetector:
    """
    Handles hand gesture detection using MediaPipe Hands.
    """
    
    def __init__(self, max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.7):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        
        # Define gestures
        self.GESTURES = {
            'POINTING': 'Pointing',
            'GRAB': 'Grab',
            'OPEN': 'Open Hand',
            'FIST': 'Fist',
            'PEACE': 'Peace',
            'THUMB_UP': 'Thumb Up'
        }
        
        self.current_gesture = None
    
    def detect_gesture(self, hand_landmarks):
        """
        Detect gesture from hand landmarks.
        
        Args:
            hand_landmarks: MediaPipe hand landmarks
            
        Returns:
            str: Detected gesture name or None
        """
        if hand_landmarks is None:
            return None
            
        # Get landmark coordinates
        landmarks = []
        for lm in hand_landmarks.landmark:
            landmarks.append([lm.x, lm.y, lm.z])
        
        # Calculate distances and angles for gesture recognition
        gesture = self._analyze_gesture(landmarks)
        self.current_gesture = gesture
        return gesture
    
    def _analyze_gesture(self, landmarks):
        """
        Analyze landmarks to determine gesture.
        
        Args:
            landmarks: List of [x, y, z] coordinates
            
        Returns:
            str: Detected gesture name or None
        """
        if len(landmarks) < 21:  # MediaPipe hands has 21 landmarks
            return None
        
        # Get key points
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]
        
        thumb_ip = landmarks[3]  # Thumb interphalangeal joint
        index_pip = landmarks[6]  # Index proximal interphalangeal joint
        middle_pip = landmarks[10]
        ring_pip = landmarks[14]
        pinky_pip = landmarks[18]
        
        wrist = landmarks[0]
        
        # Calculate distances
        def distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)
        
        # Check for pointing gesture (only index finger extended)
        index_extended = distance(index_tip, wrist) > distance(index_pip, wrist) * 1.2
        other_fingers_closed = (
            distance(middle_tip, wrist) < distance(middle_pip, wrist) * 1.1 and
            distance(ring_tip, wrist) < distance(ring_pip, wrist) * 1.1 and
            distance(pinky_tip, wrist) < distance(pinky_pip, wrist) * 1.1 and
            distance(thumb_tip, wrist) < distance(thumb_ip, wrist) * 1.1
        )
        
        if index_extended and other_fingers_closed:
            return 'POINTING'
        
        # Check for peace sign (index and middle extended)
        middle_extended = distance(middle_tip, wrist) > distance(middle_pip, wrist) * 1.2
        if index_extended and middle_extended and other_fingers_closed:
            return 'PEACE'
        
        # Check for open hand (all fingers extended)
        all_extended = (
            index_extended and middle_extended and
            distance(ring_tip, wrist) > distance(ring_pip, wrist) * 1.2 and
            distance(pinky_tip, wrist) > distance(pinky_pip, wrist) * 1.2 and
            distance(thumb_tip, wrist) > distance(thumb_ip, wrist) * 1.1
        )
        if all_extended:
            return 'OPEN'
        
        # Check for fist (all fingers closed)
        all_closed = (
            distance(index_tip, wrist) < distance(index_pip, wrist) * 1.1 and
            distance(middle_tip, wrist) < distance(middle_pip, wrist) * 1.1 and
            distance(ring_tip, wrist) < distance(ring_pip, wrist) * 1.1 and
            distance(pinky_tip, wrist) < distance(pinky_pip, wrist) * 1.1
        )
        if all_closed:
            return 'FIST'
        
        # Check for grab (fingers partially closed)
        partially_closed = (
            distance(index_tip, wrist) < distance(index_pip, wrist) * 1.3 and
            distance(middle_tip, wrist) < distance(middle_pip, wrist) * 1.3 and
            distance(ring_tip, wrist) < distance(ring_pip, wrist) * 1.3 and
            distance(pinky_tip, wrist) < distance(pinky_pip, wrist) * 1.3
        )
        if partially_closed:
            return 'GRAB'
        
        # Check for thumb up (thumb extended, others closed)
        thumb_up = (
            distance(thumb_tip, wrist) > distance(thumb_ip, wrist) * 1.2 and
            distance(index_tip, wrist) < distance(index_pip, wrist) * 1.1 and
            distance(middle_tip, wrist) < distance(middle_pip, wrist) * 1.1 and
            distance(ring_tip, wrist) < distance(ring_pip, wrist) * 1.1 and
            distance(pinky_tip, wrist) < distance(pinky_pip, wrist) * 1.1
        )
        if thumb_up:
            return 'THUMB_UP'
        
        return None
    
    def detect_hand(self, frame):
        """
        Detect hands and gestures in the frame.
        
        Args:
            frame: Input frame (BGR)
            
        Returns:
            tuple: (center, area, gesture) or (None, 0, None) if no hand detected
        """
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            # Get the first hand
            hand_landmarks = results.multi_hand_landmarks[0]
            
            # Detect gesture
            gesture = self.detect_gesture(hand_landmarks)
            
            # Calculate hand center and area
            h, w = frame.shape[:2]
            landmarks = []
            for lm in hand_landmarks.landmark:
                x = int(lm.x * w)
                y = int(lm.y * h)
                landmarks.append([x, y])
            
            # Calculate bounding box
            x_coords = [lm[0] for lm in landmarks]
            y_coords = [lm[1] for lm in landmarks]
            center_x = int(sum(x_coords) / len(x_coords))
            center_y = int(sum(y_coords) / len(y_coords))
            
            # Calculate area (approximate)
            width = max(x_coords) - min(x_coords)
            height = max(y_coords) - min(y_coords)
            area = width * height
            
            return (center_x, center_y), area, gesture
        
        return None, 0, None
    
    def draw_hand_landmarks(self, frame, hand_landmarks, color=(255, 0, 0)):
        """
        Draw hand landmarks on the frame.
        
        Args:
            frame: Frame to draw on
            hand_landmarks: MediaPipe hand landmarks
            color: Color for drawing (BGR)
            
        Returns:
            frame: Frame with landmarks drawn
        """
        if hand_landmarks:
            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=hand_landmarks,
                connections=self.mp_hands.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=color, thickness=2, circle_radius=2),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(color=color, thickness=2)
            )
        return frame
    
    def get_current_gesture(self):
        """Get the currently detected gesture."""
        return self.current_gesture
    
    def reset(self):
        """Reset the gesture detector state."""
        self.current_gesture = None 