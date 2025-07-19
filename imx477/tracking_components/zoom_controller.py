#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import time

class ZoomController:
    """
    Handles camera zoom functionality with smooth transitions and stabilization.
    """
    
    def __init__(self, zoom_levels=None):
        if zoom_levels is None:
            self.zoom_levels = [1.0, 1.5, 2.0, 2.5, 3.0]
        else:
            self.zoom_levels = zoom_levels
        
        self.current_zoom = 1.0
        self.target_zoom = 1.0
        self.zoom_region = None  # (x, y, w, h) for zoomed region
        
        # Smooth transition parameters - very conservative for stability
        self.transition_speed = 0.02  # Very slow transition for maximum stability
        self.last_zoom_change = time.time()
        self.zoom_stable_time = 8.0  # Very long time to wait before allowing new zoom changes
        
        # Stabilization parameters
        self.zoom_center_history = []  # Store recent zoom centers for smoothing
        self.history_size = 15  # More history for better stabilization
        self.min_zoom_change_threshold = 1.0  # Very large threshold to prevent frequent changes
    
    def set_zoom(self, level):
        """
        Set target zoom level for smooth transition.
        
        Args:
            level: Zoom level (must be in zoom_levels)
        """
        if level in self.zoom_levels:
            current_time = time.time()
            
            # Only allow zoom changes if enough time has passed and change is significant
            if (current_time - self.last_zoom_change > self.zoom_stable_time and 
                abs(level - self.current_zoom) > self.min_zoom_change_threshold):
                
                self.target_zoom = level
                self.last_zoom_change = current_time
                print(f"[ZOOM] Target zoom set to {level}x (current: {self.current_zoom:.1f}x)")
            else:
                print(f"[ZOOM] Zoom change ignored - too soon or too small change")
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
    
    def _smooth_zoom_transition(self):
        """
        Smoothly transition from current zoom to target zoom.
        """
        if abs(self.target_zoom - self.current_zoom) > 0.01:
            # Smooth transition
            diff = self.target_zoom - self.current_zoom
            self.current_zoom += diff * self.transition_speed
            self.current_zoom = max(1.0, min(max(self.zoom_levels), self.current_zoom))
    
    def _stabilize_zoom_center(self, center_x, center_y):
        """
        Stabilize zoom center using moving average.
        
        Args:
            center_x: Current center X
            center_y: Current center Y
            
        Returns:
            tuple: Stabilized (center_x, center_y)
        """
        if center_x is None or center_y is None:
            return center_x, center_y
        
        # Add current center to history
        self.zoom_center_history.append((center_x, center_y))
        
        # Keep only recent history
        if len(self.zoom_center_history) > self.history_size:
            self.zoom_center_history.pop(0)
        
        # Calculate moving average for stabilization
        if len(self.zoom_center_history) >= 3:
            avg_x = sum(x for x, y in self.zoom_center_history) / len(self.zoom_center_history)
            avg_y = sum(y for x, y in self.zoom_center_history) / len(self.zoom_center_history)
            
            # Add hysteresis to prevent rapid center changes
            current_avg = (avg_x, avg_y)
            if hasattr(self, 'last_stable_center'):
                # Only change center if movement is significant
                movement = abs(current_avg[0] - self.last_stable_center[0]) + abs(current_avg[1] - self.last_stable_center[1])
                if movement < 50:  # Only update if movement is less than 50 pixels
                    return self.last_stable_center
            
            self.last_stable_center = (int(avg_x), int(avg_y))
            return self.last_stable_center
        else:
            return center_x, center_y
    
    def apply_zoom(self, frame, center_x=None, center_y=None):
        """
        Apply zoom to the frame with smooth transitions and stabilization.
        
        Args:
            frame: Input frame
            center_x: X coordinate of zoom center (optional)
            center_y: Y coordinate of zoom center (optional)
            
        Returns:
            frame: Zoomed frame
        """
        # Smooth zoom transition
        self._smooth_zoom_transition()
        
        if self.current_zoom <= 1.01:  # Small threshold to avoid unnecessary processing
            return frame
        
        h, w = frame.shape[:2]
        
        # If no center specified, use frame center
        if center_x is None:
            center_x = w // 2
        if center_y is None:
            center_y = h // 2
        
        # Stabilize zoom center
        center_x, center_y = self._stabilize_zoom_center(center_x, center_y)
        
        # Calculate zoom region with padding to reduce edge artifacts
        zoom_w = int(w / self.current_zoom)
        zoom_h = int(h / self.current_zoom)
        
        # Add small padding to reduce edge jitter
        padding = int(min(zoom_w, zoom_h) * 0.02)  # 2% padding
        zoom_w = max(zoom_w - padding, 1)
        zoom_h = max(zoom_h - padding, 1)
        
        # Calculate region bounds with boundary checking
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
        
        # Use better interpolation for smoother zoom
        if zoomed_region.size > 0:
            # Use INTER_CUBIC for better quality, INTER_LANCZOS4 for highest quality
            zoomed_frame = cv2.resize(zoomed_region, (w, h), interpolation=cv2.INTER_CUBIC)
        else:
            # Fallback to original frame if zoom region is invalid
            zoomed_frame = frame
        
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
    
    def get_target_zoom(self):
        """Get the target zoom level."""
        return self.target_zoom
    
    def get_zoom_levels(self):
        """Get available zoom levels."""
        return self.zoom_levels.copy()
    
    def is_transitioning(self):
        """Check if zoom is currently transitioning."""
        return abs(self.target_zoom - self.current_zoom) > 0.01
    
    def draw_zoom_info(self, frame):
        """
        Draw zoom information on the frame.
        
        Args:
            frame: Frame to draw on
            
        Returns:
            frame: Frame with zoom information drawn
        """
        zoom_text = f'Zoom: {self.current_zoom:.1f}x'
        if self.is_transitioning():
            zoom_text += f' → {self.target_zoom:.1f}x'
        
        cv2.putText(frame, zoom_text, 
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return frame
    
    def reset(self):
        """Reset zoom to default level."""
        self.current_zoom = 1.0
        self.target_zoom = 1.0
        self.zoom_region = None
        self.zoom_center_history.clear() 