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
        # Process frame (let process_frame handle all overlays)
        display_img, block_data = self.camera_processor.process_frame(img, self.lab_data, self.__target_color)
        #print(f"Block data: {block_data}")
        
        # Only handle pickup state here
        if block_data is not None:
            if block_data.get('detected_color', 'None') != 'None':
                self.start_pick_up = True
                self.detect_color = block_data['detected_color']
                print(f"Detected color: {block_data['detected_color']}")
            #else:
            #    print(f"Raw color: {block_data.get('color', 'None')}, Waiting for confirmation...")
        return display_img
    
    def move(self):
        """Continuous scanning at each rotation position, alternating rotation direction when limit reached."""
        scan_positions = self.arm_controller.scan_positions
        num_positions = len(scan_positions)

        # Tracking variables
        total_base_rotation = 0
        max_total_rotation = 240
        rotation_step = 30
        rotation_direction = -1  # -1 for CCW, 1 for CW (start CCW)
        current_base_rotation = 0

        while self.__isRunning:
            if self.detect_color == 'None' and not self.start_pick_up:
                # Perform a full oscillating scan at current base rotation
                for oscillation in range(3):  # 3 oscillations per base rotation
                    for sweep_direction in [1, -1]:
                        if self.detect_color != 'None' or self.start_pick_up:
                            break  # Stop if detected
                        scan_range = list(range(num_positions))
                        if sweep_direction == -1:
                            scan_range = list(reversed(scan_range))
                        for idx in scan_range:
                            pos = scan_positions[idx]
                            print(f"[Scan] Visiting position {idx+1}/{num_positions}: {pos}")
                            self.arm_controller.scan_position(pos)
                            for _ in range(3):
                                time.sleep(0.1)
                                if self.detect_color != 'None' or self.start_pick_up:
                                    print(f"[Scan] Detected color: {self.detect_color} at position {pos}")
                                    break
                            if self.detect_color != 'None' or self.start_pick_up:
                                break
                    if self.detect_color != 'None' or self.start_pick_up:
                        break  # Stop if detected

                # After 3 oscillations, rotate base in the current direction
                if self.detect_color == 'None' and not self.start_pick_up:
                    if abs(current_base_rotation + rotation_direction * rotation_step) > max_total_rotation:
                        # Reached rotation limit, reverse rotation direction
                        rotation_direction *= -1
                        print(f"[Scan] Reached max rotation. Reversing rotation direction to {rotation_direction}.")
                    else:
                        # Rotate base further in the current direction
                        self.arm_controller.rotate_base(rotation_direction * rotation_step)
                        current_base_rotation += rotation_direction * rotation_step
                        print(f"[Scan] Rotated base to {current_base_rotation} degrees in direction {rotation_direction}.")
                else:
                    # Detected during scan, stop further rotation
                    break

            # Small delay to reduce CPU load
            time.sleep(0.05)

def main():
    """Main entry point."""

        
    color_palletizing = ColorPalletizing()
    color_palletizing.init()
    color_palletizing.start()

    # for pos in [1500, 1200, 1800, 1500]:
    #     color_palletizing.arm_controller.board.pwm_servo_set_position(0.5, [[6, pos]])
    #     print(f"Set base servo to {pos}")
    #     time.sleep(2)

    # Start movement control in a separate thread
    move_thread = threading.Thread(target=color_palletizing.move)
    move_thread.daemon = True
    move_thread.start()
    
    try:
        while True:
            frame = color_palletizing.camera_processor.get_frame()
            if frame is not None:
                frame_rgb = frame[..., ::-1]
                processed_frame = color_palletizing.run(frame_rgb)
                cv2.imshow('Color Palletizing', processed_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    print("Exiting program...")
                    break
                elif key == ord('c'):
                    color_palletizing.camera_processor.calibrate()
                elif key == ord('r'):
                    color_palletizing.detect_color = 'None'
                    color_palletizing.start_pick_up = False
                    color_palletizing.camera_processor.reset_scan()
                    print("[Scan] Rescan triggered by user.")
            else:
                time.sleep(0.01)
    finally:
        color_palletizing.stop()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Wait for window to close

if __name__ == '__main__':
    main() 