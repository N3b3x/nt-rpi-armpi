#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np

class ZoomController:
    """
    Handles camera zoom functionality.
    """
    
    def __init__(self, zoom_levels=None):
        if zoom_levels is None:
            self.zoom_levels = [1.0, 1.5, 2.0, 2.5, 3.0]
        else:
            self.zoom_levels = zoom_levels
        
        self.current_zoom = 1.0
        self.zoom_region = None  # (x, y, w, h) for zoomed region
    
    def set_zoom(self, level):
        """
        Set zoom level.
        
        Args:
            level: Zoom level (must be in zoom_levels)
        """
        if level in self.zoom_levels:
            self.current_zoom = level
            print(f"[ZOOM] Set zoom level to {level}x")
        else:
            print(f"[ZOOM] Invalid zoom level: {level}")
    
    def cycle_zoom(self):
        """
        Cycle through zoom levels.
        
        Returns:
            float: New zoom level
        """
        current_index = self.zoom_levels.index(self.current_zoom)
        next_index = (current_index + 1) % len(self.zoom_levels)
        self.set_zoom(self.zoom_levels[next_index])
        return self.current_zoom
    
    def apply_zoom(self, frame, center_x=None, center_y=None):
        """
        Apply zoom to the frame.
        
        Args:
            frame: Input frame
            center_x: X coordinate of zoom center (optional)
            center_y: Y coordinate of zoom center (optional)
            
        Returns:
            frame: Zoomed frame
        """
        if self.current_zoom <= 1.0:
            return frame
        
        h, w = frame.shape[:2]
        
        # If no center specified, use frame center
        if center_x is None:
            center_x = w // 2
        if center_y is None:
            center_y = h // 2
        
        # Calculate zoom region
        zoom_w = int(w / self.current_zoom)
        zoom_h = int(h / self.current_zoom)
        
        # Calculate region bounds
        x1 = max(0, center_x - zoom_w // 2)
        y1 = max(0, center_y - zoom_h // 2)
        x2 = min(w, x1 + zoom_w)
        y2 = min(h, y1 + zoom_h)
        
        # Adjust if region goes outside frame
        if x2 - x1 < zoom_w:
            if x1 == 0:
                x2 = min(w, zoom_w)
            else:
                x1 = max(0, w - zoom_w)
        if y2 - y1 < zoom_h:
            if y1 == 0:
                y2 = min(h, zoom_h)
            else:
                y1 = max(0, h - zoom_h)
        
        # Extract zoomed region
        zoomed_region = frame[y1:y2, x1:x2]
        
        # Resize to original frame size
        zoomed_frame = cv2.resize(zoomed_region, (w, h), interpolation=cv2.INTER_LINEAR)
        
        # Store zoom region for coordinate mapping
        self.zoom_region = (x1, y1, x2 - x1, y2 - y1)
        
        return zoomed_frame
    
    def map_coordinates(self, x, y):
        """
        Map coordinates from zoomed frame back to original frame.
        
        Args:
            x: X coordinate in zoomed frame
            y: Y coordinate in zoomed frame
            
        Returns:
            tuple: (x, y) coordinates in original frame
        """
        if self.zoom_region is None or self.current_zoom <= 1.0:
            return x, y
        
        x1, y1, w, h = self.zoom_region
        
        # Map coordinates back to original frame
        original_x = x1 + int(x / self.current_zoom)
        original_y = y1 + int(y / self.current_zoom)
        
        return original_x, original_y
    
    def get_current_zoom(self):
        """Get the current zoom level."""
        return self.current_zoom
    
    def get_zoom_levels(self):
        """Get available zoom levels."""
        return self.zoom_levels.copy()
    
    def draw_zoom_info(self, frame):
        """
        Draw zoom information on the frame.
        
        Args:
            frame: Frame to draw on
            
        Returns:
            frame: Frame with zoom information drawn
        """
        cv2.putText(frame, f'Zoom: {self.current_zoom}x', 
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return frame
    
    def reset(self):
        """Reset zoom to default level."""
        self.current_zoom = 1.0
        self.zoom_region = None 