#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading
import os
import cv2

# Add SDK paths
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')

import common.yaml_handle as yaml_handle
from block_tracker import BlockTracker
from arm_controller import ArmController
from camera_processor import CameraProcessor

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class ColorPalletizing:
    """
    Main class for color palletizing operation.
    """
    def __init__(self):
        # Initialize components
        self.block_tracker = BlockTracker()
        self.arm_controller = ArmController()
        self.camera_processor = CameraProcessor()
        
        # Load configuration
        self.load_config()
        
        # Initialize state variables
        self.__isRunning = False
        self._stop = False
        self.start_pick_up = False
        self.detect_color = 'None'
        self.draw_color = (0, 0, 0)
        
        # Set target colors
        self.__target_color = ('red', 'green', 'blue')
    
    def load_config(self):
        """Load configuration from yaml files."""
        # Load lab configuration
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
        print("Loaded lab_data:", self.lab_data)  # Print lab_data contents
        
        # Load coordinates
        coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)
        self.arm_controller.set_coordinates(
            coordinates_data['X'],
            coordinates_data['Y'],
            coordinates_data['Z']
        )
    
    def init(self):
        """Initialize the system."""
        print("Color Palletizing Init")
        self.arm_controller.init_move()
    
    def start(self):
        """Start the palletizing operation."""
        self.__isRunning = True
        print("Color Palletizing Start")
    
    def stop(self):
        """Stop the palletizing operation."""
        self._stop = True
        self.__isRunning = False
        print("Color Palletizing Stop")
    
    def exit(self):
        """Exit the program."""
        self._stop = True
        self.__isRunning = False
        print("Color Palletizing Exit")
    
    def set_target_color(self, target_color):
        """Set the target colors for detection."""
        print("COLOR", target_color)
        self.__target_color = target_color
        return (True, ())
    
    def run(self, img):
        """
        Run the color palletizing process.
        
        Args:
            img: Input frame from camera
            
        Returns:
            processed_frame: Frame with detection information drawn
        """
        if not self.__isRunning:
            return img
            
        # Process frame
        display_img, block_data = self.camera_processor.process_frame(img, self.lab_data, self.__target_color)
        
        # Draw detection if block found
        if block_data is not None:
            display_img = self.camera_processor.draw_detection(display_img, block_data, 0)
            # Use black color for 'None' text
            draw_color = self.camera_processor.range_rgb.get(block_data['detected_color'], (0, 0, 0))
            display_img = self.camera_processor.draw_color_text(
                display_img, 
                block_data['detected_color'],
                draw_color
            )
            
            # Start pick up if color confirmed
            if block_data['detected_color'] != 'None':
                self.start_pick_up = True
                self.detect_color = block_data['detected_color']
                print(f"Detected color: {block_data['detected_color']}")
            else:
                print(f"Raw color: {block_data['color']}, Waiting for confirmation...")
        else:
            # Always show color text, even when no block is detected
            display_img = self.camera_processor.draw_color_text(
                display_img,
                'None',
                self.camera_processor.range_rgb['black']
            )
        
        return display_img
    
    def move(self):
        """Main movement control loop."""
        while True:
            if self.__isRunning:
                # Workspace sweep logic
                if self.detect_color == 'None' and not self.start_pick_up:
                    for pos in self.arm_controller.scan_positions:
                        # Move arm to scan position
                        self.arm_controller.scan_position(pos)
                        # Let the camera process a few frames at this position
                        for _ in range(3):
                            time.sleep(0.1)
                            # If a block is detected, break out of sweep
                            if self.detect_color != 'None' or self.start_pick_up:
                                break
                        if self.detect_color != 'None' or self.start_pick_up:
                            break
                    # If still nothing detected, idle briefly
                    if self.detect_color == 'None' and not self.start_pick_up:
                        time.sleep(0.1)
                    continue
                
                if self.detect_color != 'None' and self.start_pick_up:
                    # Set RGB color
                    self.arm_controller.set_rgb(self.detect_color)
                    self.arm_controller.board.set_buzzer(1900, 0.1, 0.9, 1)
                    
                    # Pick up block
                    x, y, z = self.arm_controller.coordinates['capture']
                    self.arm_controller.pick_up_block(x, y, z)
                    
                    # Place block
                    self.arm_controller.place_block()
                    
                    # Reset state variables
                    self.detect_color = 'None'
                    self.start_pick_up = False
                    self.arm_controller.set_rgb(self.detect_color)
                else:
                    time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    self.init()
                time.sleep(0.01)

def main():
    """Main entry point."""
    color_palletizing = ColorPalletizing()
    color_palletizing.init()
    color_palletizing.start()
    
    # Start movement control in a separate thread
    # move_thread = threading.Thread(target=color_palletizing.move)
    # move_thread.daemon = True
    # move_thread.start()
    
    try:
        while True:
            frame = color_palletizing.camera_processor.get_frame()
            if frame is not None:
                processed_frame = color_palletizing.run(frame)
                cv2.imshow('Color Palletizing', processed_frame)
                
                # Check for key press
                key = cv2.waitKey(1) & 0xFF  # Get the key code
                if key == 27 or key == ord('q'):  # ESC or 'q' to quit
                    print("Exiting program...")
                    break
            else:
                time.sleep(0.01)
    finally:
        color_palletizing.stop()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Wait for window to close

if __name__ == '__main__':
    main() 