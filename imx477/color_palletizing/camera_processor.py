#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np
import sys
import os
import math

# Add parent directory to path to import Camera
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Camera import Camera
import common.misc as Misc

class CameraProcessor:
    """
    Handles camera operations and image processing.
    """
    def __init__(self, resolution=(640, 480)):
        self.camera = Camera(resolution=resolution)
        self.size = (640, 480)
        self.color_list = []
        self.detect_color = 'None'
        self.start_pick_up = False
        
        # Color mapping for display
        self.range_rgb = {
            'red': (0, 0, 255),    # BGR for red
            'blue': (255, 0, 0),   # BGR for blue
            'green': (0, 255, 0),  # BGR for green
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
    
    def get_frame(self):
        """Get a frame from the camera."""
        return self.camera.get_frame()
    
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        areaMaxContour = None
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    areaMaxContour = c
        return areaMaxContour, contour_area_max
    
    def process_frame(self, frame, lab_data, target_colors):
        if frame is None:
            return None, None
        display_img = cv2.resize(frame, self.size)
        current_color = 'None'
        draw_color = self.range_rgb['black']
        block_data = {'color': 'None', 'detected_color': self.detect_color}

        # Even during pickup, always detect live color
        frame_resize = cv2.resize(frame.copy(), self.size, interpolation=cv2.INTER_NEAREST)
        frame_lab = cv2.cvtColor(cv2.GaussianBlur(frame_resize, (3, 3), 3), cv2.COLOR_BGR2LAB)
        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        for i in lab_data:
            if i in target_colors:
                mask = cv2.inRange(frame_lab, tuple(lab_data[i]['min']), tuple(lab_data[i]['max']))
                cv2.imshow(f"Mask - {i}", mask)
                closed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                areaMaxContour, area = self.getAreaMaxContour(contours)
                if areaMaxContour is not None and area > max_area:
                    max_area = area
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour
        if max_area > 500:
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)
            cv2.circle(display_img, (int(center_x), int(center_y)), int(radius), self.range_rgb[color_area_max], 2)
            current_color = color_area_max
            draw_color = self.range_rgb.get(current_color, (0, 0, 0))
            if not self.start_pick_up:
                self.color_list.append({'red': 1, 'green': 2, 'blue': 3}.get(color_area_max, 0))
                if len(self.color_list) == 3:
                    color = int(round(np.mean(self.color_list)))
                    self.color_list.clear()
                    if color == 1:
                        self.detect_color = 'red'
                        self.start_pick_up = True
                    elif color == 2:
                        self.detect_color = 'green'
                        self.start_pick_up = True
                    elif color == 3:
                        self.detect_color = 'blue'
                        self.start_pick_up = True
                    else:
                        self.detect_color = 'None'
        else:
            current_color = 'None'
            draw_color = self.range_rgb['black']

        # Always show live detected color in overlay
        cv2.putText(display_img, f"Detected Color (live): {current_color}",
                    (10, display_img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        cv2.putText(display_img, f"Confirmed Color (pickup): {self.detect_color}",
                    (10, display_img.shape[0] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.range_rgb.get(self.detect_color, (0, 0, 0)), 2)
        cv2.putText(display_img, "c-recalibrate | q-quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        block_data['color'] = current_color
        block_data['detected_color'] = self.detect_color

        return display_img, block_data

    def reset(self):
        """Reset the camera processor state."""
        self.color_list = []
        self.detect_color = 'None'
        self.start_pick_up = False

    def draw_detection(self, display_img, block_data, block_id):
        """
        Draw detection information on the display image.
        
        Args:
            display_img: Image to draw on
            block_data: Dictionary containing block information
            block_id: ID of the block
            
        Returns:
            display_img: Image with detection information drawn
        """
        center_x, center_y = block_data['position']
        radius = block_data['radius']
        color = block_data['color']
        
        # Draw circle around block
        cv2.circle(display_img, (center_x, center_y), int(radius), 
                  self.range_rgb[color], 2)
        
        # Draw block ID and color
        cv2.putText(display_img, f"{color} ({block_id})", 
                   (center_x - 20, center_y - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color], 2)
        
        return display_img
    
    def draw_color_text(self, display_img, detect_color, draw_color):
        """
        Draw the current detected color text on the display image.
        
        Args:
            display_img: Image to draw on
            detect_color: Current detected color
            draw_color: Color to use for drawing
            
        Returns:
            display_img: Image with color text drawn
        """
        cv2.putText(display_img, "Color: " + detect_color,
                   (10, display_img.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        return display_img
    
    def detect_color(self, block_data):
        """
        Detect and convert block color to numeric value.
        
        Args:
            block_data: Dictionary containing block information
            
        Returns:
            tuple: (detect_color, draw_color, start_pick_up)
        """
        if block_data['area'] > 500:
            # Convert color to numeric value
            if block_data['color'] == 'red':
                color = 1
            elif block_data['color'] == 'green':
                color = 2
            elif block_data['color'] == 'blue':
                color = 3
            else:
                color = 0
            
            # Update color detection
            return self.update_color_list(color)
        
        return 'None', self.range_rgb["black"], False
    
    def update_color_list(self, color):
        """
        Update the color list for color confirmation.
        
        Args:
            color: Numeric color value (1=red, 2=green, 3=blue)
            
        Returns:
            tuple: (detect_color, draw_color, start_pick_up)
        """
        self.color_list.append(color)
        if len(self.color_list) == 3:
            color = int(round(np.mean(np.array(self.color_list))))
            self.color_list = []
            
            if color == 1:
                return 'red', self.range_rgb["red"], True
            elif color == 2:
                return 'green', self.range_rgb["green"], True
            elif color == 3:
                return 'blue', self.range_rgb["blue"], True
            else:
                return 'None', self.range_rgb["black"], False
        
        return 'None', self.range_rgb["black"], False

    def detect_blocks(self, frame_lab, lab_data, target_colors):
        """
        Detect blocks in the frame using LAB color space.
        
        Args:
            frame_lab: Frame in LAB color space
            lab_data: LAB color range data
            target_colors: List of target colors to detect
            
        Returns:
            dict: Dictionary of detected blocks with their properties
        """
        detected_blocks = {}
        img_h, img_w = frame_lab.shape[:2]
        
        # Track the largest area for each color
        max_area = 0
        color_area_max = None
        areaMaxContour_max = None
        
        for color in target_colors:
            if color in lab_data:
                # Get LAB color range
                min_l, min_a, min_b = lab_data[color]['min']
                max_l, max_a, max_b = lab_data[color]['max']
                
                # Create mask for color
                frame_mask = cv2.inRange(
                    frame_lab,
                    (min_l, min_a, min_b),
                    (max_l, max_a, max_b)
                )
                
                # Apply morphological operations
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                
                # Ignore top and left edges
                closed[0:80, :] = 0
                closed[:, 0:120] = 0
                
                # Find contours
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                
                # Find largest contour
                areaMaxContour = None
                area_max = 0
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > area_max and area > 300:
                        area_max = area
                        areaMaxContour = contour
                
                # Update if this color has the largest area
                if areaMaxContour is not None and area_max > max_area:
                    max_area = area_max
                    color_area_max = color
                    areaMaxContour_max = areaMaxContour
        
        # Process the largest detected block
        if max_area > 500 and color_area_max is not None and areaMaxContour_max is not None:
            # Get center and radius
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)
            
            # Map coordinates to image space
            center_x = int(Misc.map(center_x, 0, self.size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, self.size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
            
            # Add to detected blocks
            detected_blocks[0] = {
                'position': (center_x, center_y),
                'area': max_area,
                'radius': radius,
                'color': color_area_max
            }
            
            # Update color list for confirmation
            if color_area_max == 'red':
                color = 1
            elif color_area_max == 'green':
                color = 2
            elif color_area_max == 'blue':
                color = 3
            else:
                color = 0
            
            self.color_list.append(color)
            if len(self.color_list) == 3:
                color = int(round(np.mean(np.array(self.color_list))))
                self.color_list = []
                
                if color == 1:
                    detected_blocks[0]['detected_color'] = 'red'
                elif color == 2:
                    detected_blocks[0]['detected_color'] = 'green'
                elif color == 3:
                    detected_blocks[0]['detected_color'] = 'blue'
                else:
                    detected_blocks[0]['detected_color'] = 'None'
            else:
                detected_blocks[0]['detected_color'] = 'None'
        
        return detected_blocks 

    def calibrate(self):
        """Placeholder for calibration logic (to be called from main on 'c' key)."""
        print("Calibration triggered (implement as needed)") 

    def reset_scan(self):
        """Reset scan state for rescanning."""
        self.color_list = []
        self.detect_color = 'None'
        self.start_pick_up = False 