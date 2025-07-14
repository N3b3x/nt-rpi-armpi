#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import mediapipe as mp
import math
import pickle
import os

class FaceDetector:
    """
    Handles face detection and recognition using MediaPipe.
    """
    
    def __init__(self, max_num_faces=1, min_detection_confidence=0.8, min_tracking_confidence=0.8):
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_face_mesh = mp.solutions.face_mesh
        
        self.face_detection = self.mp_face_detection.FaceDetection(
            min_detection_confidence=min_detection_confidence
        )
        
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=max_num_faces,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        
        # Simple face recognition database
        self.known_faces = {}  # Dictionary to store known face features
        self.face_db_path = 'known_faces.pkl'
        self.load_face_database()
        
        self.recognized_face = None
    
    def load_face_database(self):
        """Load known faces from database file."""
        if os.path.exists(self.face_db_path):
            try:
                with open(self.face_db_path, 'rb') as f:
                    self.known_faces = pickle.load(f)
                print(f"[INFO] Loaded {len(self.known_faces)} known faces")
            except Exception as e:
                print(f"[WARNING] Failed to load face database: {e}")
                self.known_faces = {}
    
    def save_face_database(self):
        """Save known faces to database file."""
        try:
            with open(self.face_db_path, 'wb') as f:
                pickle.dump(self.known_faces, f)
            print(f"[INFO] Saved {len(self.known_faces)} known faces")
        except Exception as e:
            print(f"[ERROR] Failed to save face database: {e}")
    
    def add_face(self, name, face_img):
        """
        Add a new face to the database.
        
        Args:
            name: Name of the person
            face_img: Face image (BGR)
        """
        features = self._extract_face_features(face_img)
        if features is not None:
            self.known_faces[name] = features
            self.save_face_database()
            print(f"[INFO] Added face for {name}")
            return True
        return False
    
    def _extract_face_features(self, face_img):
        """
        Extract face features using MediaPipe FaceMesh.
        
        Args:
            face_img: Face image (BGR)
            
        Returns:
            numpy.ndarray: Face features or None
        """
        # Convert BGR to RGB
        rgb_img = cv2.cvtColor(face_img, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe FaceMesh
        results = self.face_mesh.process(rgb_img)
        
        if results.multi_face_landmarks:
            # Get the first face
            face_landmarks = results.multi_face_landmarks[0]
            
            # Extract key facial features (simplified)
            features = []
            h, w = face_img.shape[:2]
            
            # Use key landmarks for feature extraction
            key_landmarks = [10, 33, 61, 93, 152, 234, 454, 468]  # Key facial points
            
            for landmark_id in key_landmarks:
                if landmark_id < len(face_landmarks.landmark):
                    lm = face_landmarks.landmark[landmark_id]
                    # Normalize coordinates
                    features.extend([lm.x, lm.y, lm.z])
            
            return np.array(features)
        
        return None
    
    def _recognize_face_mediapipe(self, face_img):
        """
        Recognize face using MediaPipe features.
        
        Args:
            face_img: Face image (BGR)
            
        Returns:
            str: Recognized name or 'Unknown'
        """
        features = self._extract_face_features(face_img)
        if features is None:
            return 'Unknown'
        
        best_match = None
        best_distance = float('inf')
        
        for name, known_features in self.known_faces.items():
            # Calculate Euclidean distance
            distance = np.linalg.norm(features - known_features)
            
            if distance < best_distance and distance < 0.1:  # Threshold for recognition
                best_distance = distance
                best_match = name
        
        return best_match if best_match else 'Unknown'
    
    def detect_face(self, frame):
        """
        Detect faces in the frame.
        
        Args:
            frame: Input frame (BGR)
            
        Returns:
            tuple: (center, area, name) or (None, 0, None) if no face detected
        """
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe FaceDetection
        results = self.face_detection.process(rgb_frame)
        
        if results.detections:
            # Get the first face
            detection = results.detections[0]
            
            # Get bounding box
            bbox = detection.location_data.relative_bounding_box
            h, w = frame.shape[:2]
            
            x = int(bbox.xmin * w)
            y = int(bbox.ymin * h)
            width = int(bbox.width * w)
            height = int(bbox.height * h)
            
            # Calculate center and area
            center_x = x + width // 2
            center_y = y + height // 2
            area = width * height
            
            # Extract face region for recognition
            face_img = frame[y:y+height, x:x+width]
            if face_img.size > 0:
                name = self._recognize_face_mediapipe(face_img)
                self.recognized_face = name
            else:
                name = 'Unknown'
                self.recognized_face = None
            
            return (center_x, center_y), area, name
        
        self.recognized_face = None
        return None, 0, None
    
    def draw_face_detection(self, frame, center, area, name, color=(0, 255, 0)):
        """
        Draw face detection on the frame.
        
        Args:
            frame: Frame to draw on
            center: Face center coordinates (x, y)
            area: Face area
            name: Recognized name
            color: Color for drawing (BGR)
            
        Returns:
            frame: Frame with face detection drawn
        """
        if center is not None and area > 0:
            x, y = center
            size = int(math.sqrt(area))
            half_size = size // 2
            
            # Draw rectangle around face
            cv2.rectangle(frame, (x - half_size, y - half_size), 
                         (x + half_size, y + half_size), color, 2)
            
            # Draw name
            if name and name != 'Unknown':
                cv2.putText(frame, f'Face: {name}', 
                           (x - half_size, y - half_size - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        
        return frame
    
    def get_recognized_face(self):
        """Get the currently recognized face."""
        return self.recognized_face
    
    def reset(self):
        """Reset the face detector state."""
        self.recognized_face = None 