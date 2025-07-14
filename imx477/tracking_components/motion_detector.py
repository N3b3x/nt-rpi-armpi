#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import time

class MotionDetector:
    """
    Handles motion detection and spotlight control.
    """
    
    def __init__(self, motion_threshold=25, motion_area_threshold=500, 
                 motion_history=5, motion_timeout=5):
        self.motion_threshold = motion_threshold
        self.motion_area_threshold = motion_area_threshold
        self.motion_history = motion_history
        self.motion_timeout = motion_timeout
        
        self.motion_frames = []  # List to store previous frames
        self.last_motion_time = 0
        
        # Spotlight control
        self.spotlight_on = False
        self.spotlight_brightness = 100  # 0-100
        self.spotlight_color = (255, 255, 255)  # RGB color
    
    def detect_motion(self, frame):
        """
        Detect motion in the frame.
        
        Args:
            frame: Input frame (BGR)
            
        Returns:
            tuple: (motion_center, motion_area) or (None, 0) if no motion
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # Add current frame to history
        self.motion_frames.append(gray)
        if len(self.motion_frames) > self.motion_history:
            self.motion_frames.pop(0)
        
        # Need at least 2 frames to detect motion
        if len(self.motion_frames) < 2:
            return None, 0
        
        # Calculate difference between current and previous frame
        frame_delta = cv2.absdiff(self.motion_frames[-2], self.motion_frames[-1])
        thresh = cv2.threshold(frame_delta, self.motion_threshold, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate the thresholded image to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours of motion
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.motion_area_threshold:
                # Get the bounding box of the motion
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w//2
                center_y = y + h//2
                self.last_motion_time = time.time()
                return (center_x, center_y), area
        
        return None, 0
    
    def check_motion_timeout(self):
        """
        Check if motion has timed out and turn off spotlight if needed.
        
        Returns:
            bool: True if spotlight was turned off due to timeout
        """
        if self.spotlight_on and time.time() - self.last_motion_time > self.motion_timeout:
            self.control_spotlight(False)
            return True
        return False
    
    def control_spotlight(self, on=True, brightness=100, color=(255, 255, 255)):
        """
        Control the spotlight mounted on the arm.
        
        Args:
            on: Whether to turn spotlight on
            brightness: Brightness level (0-100)
            color: RGB color tuple
        """
        self.spotlight_on = on
        self.spotlight_brightness = brightness
        self.spotlight_color = color
        
        # Note: This would interface with the board controller
        # For now, we just track the state
        if on:
            print(f"[MOTION] Spotlight ON - Brightness: {brightness}, Color: {color}")
        else:
            print("[MOTION] Spotlight OFF")
    
    def draw_motion_detection(self, frame, motion_center, motion_area):
        """
        Draw motion detection on the frame.
        
        Args:
            frame: Frame to draw on
            motion_center: Motion center coordinates (x, y)
            motion_area: Motion area
            
        Returns:
            frame: Frame with motion detection drawn
        """
        if motion_center is not None:
            # Draw motion detection circle
            cv2.circle(frame, motion_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, 'Motion Detected', 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        # Draw spotlight status
        spotlight_status = "ON" if self.spotlight_on else "OFF"
        cv2.putText(frame, f'Spotlight: {spotlight_status}', 
                    (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        
        return frame
    
    def is_spotlight_on(self):
        """Check if spotlight is currently on."""
        return self.spotlight_on
    
    def reset(self):
        """Reset the motion detector state."""
        self.motion_frames = []
        self.last_motion_time = 0
        self.spotlight_on = False
        self.control_spotlight(False) 