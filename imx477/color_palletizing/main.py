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
        """Main movement control loop."""
        sweep_direction = 1  # 1 = forward, -1 = backward
        oscillations = 0     # Count how many oscillations (back-and-forth sweeps)
        max_oscillations = 3
        last_rotation_ccw = True  # Start with CCW
        total_base_rotation = 0
        max_total_rotation = 240  # Stop after total rotation of 240 degrees

        while True:
            if self.__isRunning:
                if self.detect_color == 'None' and not self.start_pick_up:
                    # Perform a single oscillation sweep
                    scan_range = list(range(len(self.arm_controller.scan_positions)))
                    if sweep_direction == -1:
                        scan_range = list(reversed(scan_range))

                    for idx in scan_range:
                        pos = self.arm_controller.scan_positions[idx]
                        print(f"[Scan] Visiting position {idx+1}/{len(self.arm_controller.scan_positions)}: {pos}")
                        self.arm_controller.scan_position(pos)
                        for _ in range(3):
                            time.sleep(0.1)
                            if self.detect_color != 'None' or self.start_pick_up:
                                print(f"[Scan] Detected color: {self.detect_color} at position {pos}")
                                break
                        if self.detect_color != 'None' or self.start_pick_up:
                            break
                    else:
                        # One oscillation done
                        sweep_direction *= -1
                        oscillations += 1
                        print(f"[Scan] Completed {oscillations}/{max_oscillations} oscillations at this base rotation.")

                    # After 3 oscillations, rotate base and reset oscillation count
                    if oscillations >= max_oscillations:
                        if total_base_rotation < max_total_rotation:
                            if last_rotation_ccw:
                                print("[Scan] No detection. Rotating base anti-clockwise by 30 degrees.")
                                self.arm_controller.rotate_base(-30)
                                total_base_rotation += 30
                            else:
                                print("[Scan] No detection. Rotating base clockwise by 30 degrees.")
                                self.arm_controller.rotate_base(30)
                                total_base_rotation += 30
                            last_rotation_ccw = not last_rotation_ccw
                            oscillations = 0
                            #sweep_direction = 1  # Always start new sweep forward
                            print(f"[Scan] Total base rotation: {total_base_rotation} degrees.")
                        else:
                            print("[Scan] Max total rotation (240 degrees) reached. Stopping scan.")
                            self.__isRunning = False
                            break

                    # If still nothing detected, idle briefly
                    if self.detect_color == 'None' and not self.start_pick_up:
                        time.sleep(0.1)
                    continue

                if self.detect_color != 'None' and self.start_pick_up:
                    # --- Pick and place logic  ---
                    # self.arm_controller.set_rgb(self.detect_color)
                    # self.arm_controller.board.set_buzzer(1900, 0.1, 0.9, 1)
                    # x, y, z = self.arm_controller.coordinates['capture']
                    # self.arm_controller.pick_up_block(x, y, z)
                    # self.arm_controller.place_block()
                    # self.detect_color = 'None'
                    # self.start_pick_up = False
                    # self.arm_controller.set_rgb(self.detect_color)
                    # --- End pick and place logic ---

                    # Instead, just stop here for now
                    print(f"Block detected: {self.detect_color}. Stopping scan.")
                    while self.detect_color != 'None' and self.start_pick_up:
                        time.sleep(0.1)
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