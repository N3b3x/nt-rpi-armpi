#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import math

class DepthEstimator:
    """
    Handles depth estimation and visualization.
    """
    
    def __init__(self, reference_distance=30, reference_size=1000, depth_history=10):
        self.reference_distance = reference_distance  # Reference distance in cm
        self.reference_size = reference_size  # Reference size at reference_distance
        self.depth_history = depth_history
        
        self.depth_history_list = []  # List to store previous depth measurements
        self.current_depth = None
        self.depth_confidence = 0.0
        
        # Depth thresholds for color coding
        self.CLOSE_THRESHOLD = 20  # cm
        self.FAR_THRESHOLD = 50    # cm
    
    def estimate_depth(self, area, current_z):
        """
        Estimate depth based on object area and current Z position.
        
        Args:
            area: Object area in pixels
            current_z: Current Z position of the arm
            
        Returns:
            tuple: (depth, confidence)
        """
        if area <= 0:
            return None, 0.0
        
        # Simple depth estimation based on area
        # Larger area = closer object
        estimated_depth = self.reference_distance * (self.reference_size / area) ** 0.5
        
        # Add current Z position as a factor
        estimated_depth = (estimated_depth + current_z) / 2
        
        # Add to history for smoothing
        self.depth_history_list.append(estimated_depth)
        if len(self.depth_history_list) > self.depth_history:
            self.depth_history_list.pop(0)
        
        # Calculate smoothed depth and confidence
        if len(self.depth_history_list) > 0:
            self.current_depth = np.mean(self.depth_history_list)
            self.depth_confidence = 1.0 - (np.std(self.depth_history_list) / self.current_depth)
            self.depth_confidence = max(0.0, min(1.0, self.depth_confidence))
        else:
            self.current_depth = estimated_depth
            self.depth_confidence = 0.5
        
        return self.current_depth, self.depth_confidence
    
    def get_depth_color(self, depth):
        """
        Get color based on depth for visualization.
        
        Args:
            depth: Depth in cm
            
        Returns:
            tuple: BGR color
        """
        if depth is None:
            return (255, 255, 255)  # White for unknown
        
        if depth <= self.CLOSE_THRESHOLD:
            # Red to yellow gradient (close objects)
            ratio = depth / self.CLOSE_THRESHOLD
            return (0, int(255 * ratio), 255)
        elif depth <= self.FAR_THRESHOLD:
            # Yellow to green gradient (medium distance)
            ratio = (depth - self.CLOSE_THRESHOLD) / (self.FAR_THRESHOLD - self.CLOSE_THRESHOLD)
            return (int(255 * (1 - ratio)), 255, 0)
        else:
            # Green to blue gradient (far objects)
            ratio = min(1.0, (depth - self.FAR_THRESHOLD) / self.FAR_THRESHOLD)
            return (0, int(255 * (1 - ratio)), int(255 * ratio))
    
    def draw_depth_info(self, frame, depth, confidence):
        """
        Draw depth information on the frame.
        
        Args:
            frame: Frame to draw on
            depth: Depth in cm
            confidence: Confidence value (0-1)
            
        Returns:
            frame: Frame with depth information drawn
        """
        if depth is not None:
            depth_text = f'Depth: {depth:.1f}cm (Conf: {confidence:.2f})'
            cv2.putText(frame, depth_text, 
                        (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        
        return frame
    
    def get_current_depth(self):
        """Get the current estimated depth."""
        return self.current_depth
    
    def get_depth_confidence(self):
        """Get the current depth confidence."""
        return self.depth_confidence
    
    def reset(self):
        """Reset the depth estimator state."""
        self.depth_history_list = []
        self.current_depth = None
        self.depth_confidence = 0.0 