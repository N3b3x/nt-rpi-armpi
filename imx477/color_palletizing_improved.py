#!/usr/bin/python3
# coding=utf8
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
import cv2
import time
import math
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle
from Camera import Camera

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Read picking coordinates
Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

range_rgb = {
    'red': (0, 0, 255),    # BGR for red
    'blue': (255, 0, 0),   # BGR for blue
    'green': (0, 255, 0),  # BGR for green
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Block tracking class
class BlockTracker:
    def __init__(self):
        self.detected_blocks = {}  # Dictionary to store detected blocks
        self.picked_blocks = set()  # Set to track picked blocks
        self.block_positions = {}  # Dictionary to store block positions
        self.min_detection_frames = 3  # Number of frames to confirm block detection
        self.position_threshold = 10  # Maximum pixel movement to consider same block
        
    def update_block_positions(self, frame, lab_data, target_colors):
        # Convert frame to LAB color space
        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        # Process each target color
        for color in target_colors:
            if color in lab_data:
                # Create mask for current color
                frame_mask = cv2.inRange(frame_lab,
                                       tuple(lab_data[color]['min']),
                                       tuple(lab_data[color]['max']))
                
                # Apply morphological operations
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                
                # Ignore top and left edges
                closed[0:80, :] = 0
                closed[:, 0:120] = 0
                
                # Find contours
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                
                # Process each contour
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 300:  # Minimum area threshold
                        # Get center and radius
                        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
                        center_x, center_y = int(center_x), int(center_y)
                        
                        # Create block ID
                        block_id = f"{color}_{center_x}_{center_y}"
                        
                        # Update block position
                        if block_id in self.block_positions:
                            # Check if block has moved significantly
                            old_x, old_y = self.block_positions[block_id]['position']
                            if abs(center_x - old_x) < self.position_threshold and \
                               abs(center_y - old_y) < self.position_threshold:
                                self.block_positions[block_id]['detection_count'] += 1
                                if self.block_positions[block_id]['detection_count'] >= self.min_detection_frames:
                                    self.detected_blocks[block_id] = {
                                        'color': color,
                                        'position': (center_x, center_y),
                                        'radius': radius,
                                        'area': area
                                    }
                        else:
                            # New block detected
                            self.block_positions[block_id] = {
                                'position': (center_x, center_y),
                                'detection_count': 1
                            }
        
        # Remove blocks that haven't been detected recently
        self.cleanup_old_blocks()
        
        return self.detected_blocks
    
    def cleanup_old_blocks(self):
        # Remove blocks that haven't been detected in recent frames
        current_blocks = set(self.block_positions.keys())
        for block_id in current_blocks:
            if block_id not in self.detected_blocks:
                del self.block_positions[block_id]
    
    def mark_block_as_picked(self, block_id):
        self.picked_blocks.add(block_id)
        if block_id in self.detected_blocks:
            del self.detected_blocks[block_id]
        if block_id in self.block_positions:
            del self.block_positions[block_id]
    
    def get_next_block(self, target_colors):
        # Prioritize blocks based on color order
        for color in target_colors:
            for block_id, block_data in self.detected_blocks.items():
                if block_data['color'] == color and block_id not in self.picked_blocks:
                    return block_id, block_data
        return None, None

# Read color threshold file
lab_data = None
def load_config():
    global lab_data
    print("yaml_handle loaded from:", yaml_handle.__file__)
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)

# Set target colors
__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color
    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# Initialize block tracker
block_tracker = BlockTracker()

# Define scan positions (side-to-side sweep along X axis at fixed Y, Z)
scan_positions = [
    (x, 0, 10) for x in np.linspace(5, 19, num=6)
]

# Rest of the code remains the same until the run function
def run(img):
    global roi
    global get_roi
    global center_list
    global __isRunning
    global start_pick_up
    global detect_color, draw_color, color_list
    
    if not __isRunning:
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    # Always prepare display_img
    display_img = cv2.resize(img, size)
    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_rgb = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2RGB)
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

    if not start_pick_up:
        # Update block positions
        detected_blocks = block_tracker.update_block_positions(frame_lab, lab_data, __target_color)
        
        # Get next block to pick up
        block_id, block_data = block_tracker.get_next_block(__target_color)
        
        if block_data is not None:
            # Draw detected blocks
            for bid, bdata in detected_blocks.items():
                center_x, center_y = bdata['position']
                radius = bdata['radius']
                color = bdata['color']
                cv2.circle(display_img, (center_x, center_y), int(radius), range_rgb[color], 2)
                cv2.putText(display_img, f"{color} ({bid})", (center_x - 20, center_y - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color], 2)
            
            # If we have a stable block detection
            if block_data['area'] > 500:
                if block_data['color'] == 'red':
                    color = 1
                elif block_data['color'] == 'green':
                    color = 2
                elif block_data['color'] == 'blue':
                    color = 3
                else:
                    color = 0
                
                color_list.append(color)
                if len(color_list) == 3:
                    color = int(round(np.mean(np.array(color_list))))
                    color_list = []
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                        start_pick_up = True
                        # Mark block as picked
                        block_tracker.mark_block_as_picked(block_id)
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                        start_pick_up = True
                        block_tracker.mark_block_as_picked(block_id)
                    elif color == 3:
                        start_pick_up = True
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                        block_tracker.mark_block_as_picked(block_id)
                    else:
                        start_pick_up = False
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"
    
    cv2.putText(display_img, "Color: " + detect_color, (10, display_img.shape[0] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return display_img

def move():
    global _stop, get_roi, number, __isRunning, detect_color, start_pick_up
    
    x = Coordinates_data['X']
    y = Coordinates_data['Y']
    z = Coordinates_data['Z']
    
    coordinate = {
        'capture': (x, y, z),       # Gripping coordinate
        'place': (12, 0, 0.5),      # Placing coordinate
    }

    while True:
        if __isRunning:
            # --- Workspace sweep logic ---
            if detect_color == 'None' and not start_pick_up:
                for pos in scan_positions:
                    # Move arm to scan position
                    AK.setPitchRangeMoving(pos, -90, -90, 90, 800)
                    time.sleep(0.7)  # Wait for arm to stabilize
                    # Let the camera process a few frames at this position
                    for _ in range(3):
                        time.sleep(0.1)
                        # If a block is detected, break out of sweep
                        if detect_color != 'None' or start_pick_up:
                            break
                    if detect_color != 'None' or start_pick_up:
                        break
                # If still nothing detected, idle briefly
                if detect_color == 'None' and not start_pick_up:
                    time.sleep(0.1)
                continue
            # --- End sweep logic ---
            if detect_color != 'None' and start_pick_up:
                set_rgb(detect_color)
                board.set_buzzer(1900, 0.1, 0.9, 1)
                board.pwm_servo_set_position(0.5, [[1, 1900]])  # Open gripper - less extreme
                time.sleep(0.8)
                if not __isRunning: continue
                # Use the last scan position for pickup (or could use detected block position if mapped)
                pick_z = z - 2.0  # Go 2cm lower for pickup
                AK.setPitchRangeMoving((x, y, pick_z), -90, -90, 90, 1000)
                time.sleep(1)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[1, 1500]])  # Close gripper - tighter
                time.sleep(0.8)
                if not __isRunning: continue
                AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1500)
                time.sleep(1.5)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])
                time.sleep(1.5)
                if not __isRunning: continue
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], 12), -90, -90, 90, 800)
                time.sleep(0.8)
                if not __isRunning: continue
                # Adjusted stacking heights
                if number == 0:  # First block - 0.5cm lower
                    place_z = coordinate['place'][2] - 0.5
                elif number == 1:  # Second block - 0.5cm lower
                    place_z = coordinate['place'][2] + 2
                else:  # Third block - unchanged
                    place_z = coordinate['place'][2] + 2.5
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], place_z), -90, -90, 90, 800)
                time.sleep(0.8)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[1, 1900]])  # Open gripper - less extreme
                time.sleep(0.8)
                if not __isRunning: continue
                AK.setPitchRangeMoving((6, 0, 18), 0, -90, 90, 1500)
                time.sleep(1.5)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])
                time.sleep(1.5)
                if not __isRunning: continue
                AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 800)
                time.sleep(0.8)
                number += 1
                if number == 3:
                    number = 0
                    board.set_buzzer(1900, 0.1, 0.9, 1)
                    set_rgb('white')
                    time.sleep(0.5)
                if not __isRunning: continue
                # Reset state variables
                detect_color = 'None'
                get_roi = False
                start_pick_up = False
                set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                initMove()
            time.sleep(0.01)

# Rest of the code remains the same 