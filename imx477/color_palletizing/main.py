#!/usr/bin/python3
# coding=utf8
# main.py

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
                #print(f"Detected color: {block_data['detected_color']}")
            #else:
            #    print(f"Raw color: {block_data.get('color', 'None')}, Waiting for confirmation...")
        return display_img
    
    def handle_pickup_and_place(self):
        """
        Execute the pickup and placement sequence.
        """
        print(f"Block detected: {self.detect_color}. Initiating pickup sequence.")
        self.arm_controller.set_rgb(self.detect_color)
        self.arm_controller.board.set_buzzer(1900, 0.1, 0.9, 1)

        # Use the last detected scan position (x, y) and a safe pickup Z
        x, y, _ = self.arm_controller.coordinates['capture']
        pickup_z = 5  # 🟢 Lower Z to actually pick block!

        print(f"[Pickup] Moving to pickup coordinates: ({x}, {y}, {pickup_z})")
        self.arm_controller.pick_up_block(x, y, pickup_z)
        self.arm_controller.place_block()

        self.detect_color = 'None'
        self.start_pick_up = False
        self.arm_controller.set_rgb(self.detect_color)

        print("[Pickup] Pickup & place complete. Resuming scan.")

    def move(self):
        """
        Enhanced polar scan with ±180° range, with immediate pickup after detection.
        """
        angles = list(range(-180, 181, 75))  # Rotation angles to cover ±180°
        angle_idx = 0
        fixed_pitch = -10  # Slight downward pitch angle

        while self.__isRunning:
            if self.detect_color == 'None' and not self.start_pick_up:
                base_angle = angles[angle_idx]
                print(f"[Scan] Starting scan at base angle: {base_angle}° (fixed pitch {fixed_pitch}°)")

                # Rotate to base angle
                rotation_needed = base_angle - self.arm_controller.current_base_angle
                self.arm_controller.rotate_base(rotation_needed)
                time.sleep(0.15)  # Stabilization time

                # Get scan positions for this base angle
                scan_positions = self.arm_controller.get_scan_positions(base_angle)

                for idx, pos in enumerate(scan_positions):
                    if not self.__isRunning:
                        break  # If stopped, exit immediately

                    print(f"[Scan] Visiting position {idx+1}/{len(scan_positions)}: {pos} (fixed pitch {fixed_pitch}°)")
                    self.arm_controller.scan_position(pos, pitch_angle=fixed_pitch)

                    # Check for color detection (quick short pauses)
                    for _ in range(2):
                        time.sleep(0.025)
                        if self.detect_color != 'None':
                            print(f"[Scan] Detected color: {self.detect_color} at position {pos}")
                            self.arm_controller.set_coordinates(*pos)
                            self.start_pick_up = True  # Trigger pickup logic
                            break

                    if self.start_pick_up:
                        # 💡 Directly call pickup after detection
                        self.handle_pickup_and_place()
                        break  # Exit scan positions loop after pickup

                # Move to next angle if no detection
                if self.detect_color == 'None' and not self.start_pick_up:
                    angle_idx += 1
                    if angle_idx >= len(angles):
                        angle_idx = 0
                        print("[Scan] Completed full sweep. Restarting.")
                    else:
                        print(f"[Scan] Moving to next angle: {angles[angle_idx]}°")
            else:
                time.sleep(0.01)

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
                
                # If the scan thread has exited, restart it
                if not move_thread.is_alive():
                    print("[Scan] Restarting scanning thread.")
                    move_thread = threading.Thread(target=color_palletizing.move)
                    move_thread.daemon = True
                    move_thread.start()
            else:
                time.sleep(0.01)
    finally:
        color_palletizing.stop()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Wait for window to close

if __name__ == '__main__':
    main() 