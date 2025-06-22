#!/usr/bin/python3
# pickup_test.py
# Testing pickup sequence with varying pitch angles.

import sys
import time
import math
import logging
import yaml
import os
import cv2
import numpy as np
from common.ros_robot_controller_sdk import Board

# Add necessary paths first (so they take precedence)
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')  # Add kinematics_sdk path
sys.path.append('/home/pi/ArmPi_mini/')

# Add parent directory to path to import Camera
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print("Python path:", sys.path)
print("Current directory:", os.getcwd())

# Now import the modules that need the paths
import common.pid as PID
import common.misc as Misc

# Import yaml_handle directly from its file
yaml_handle_path = '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk/common/yaml_handle.py'
import importlib.util
spec = importlib.util.spec_from_file_location("yaml_handle", yaml_handle_path)
yaml_handle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(yaml_handle)

print("yaml_handle module path:", yaml_handle.__file__)
print("yaml_handle attributes:", dir(yaml_handle))

# Configure logging to only show INFO and above
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Path setup for local my_kinematics module
from my_kinematics.arm_move_ik import ArmIK

# Constants
SERVO_GRIPPER = 1
SERVO_BASE = 6

class PickupTest:
    def __init__(self):
        self.board = Board()
        self.AK = ArmIK()
        self.AK.board = self.board

    def open_gripper(self):
        print("[Test] Opening gripper")
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1900]])
        time.sleep(0.5)

    def close_gripper(self):
        print("[Test] Closing gripper")
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1500]])
        time.sleep(0.5)

    def move_to_position(self, pos, pitch=-90, movetime=1000):
        result = self.AK.setPitchRangeMoving(pos, pitch, pitch, 0, movetime)
        if not result:
            print(f"[WARNING] Could not reach position: {pos} with pitch {pitch}")
        else:
            time.sleep(result[2] / 1000)

    def rotate_base(self, pulse):
        print(f"[Test] Rotating base servo6 to pulse {pulse}")
        self.board.pwm_servo_set_position(0.5, [[SERVO_BASE, pulse]])
        time.sleep(0.8)

    def test_varying_pitch_angles(self):
        print("\n==== TEST: Varying Pitch Angles in Pickup Sequence ====")
        pickup_pos = (10, 0, 2)
        dropoff_z = 1.5
        base_pulse = 2000  # example base rotation

        pitch_angles = [-90, -60, -45, -30]  # angles to test

        for idx, pitch in enumerate(pitch_angles):
            print(f"\n--- Cycle {idx+1}: Pitch angle {pitch}° ---")

            # 1. Open gripper
            self.open_gripper()

            # 2. Move to pickup
            print("[Test] Moving to pickup position")
            self.move_to_position(pickup_pos, pitch=pitch)

            # 3. Close gripper
            self.close_gripper()

            # 4. Lift up
            print("[Test] Lifting up")
            self.move_to_position((pickup_pos[0], pickup_pos[1], 8), pitch=pitch)

            # 5. Rotate base
            self.rotate_base(base_pulse)

            # 6. Move above dropoff
            dropoff_pos = (10, 0, 8)
            self.move_to_position(dropoff_pos, pitch=pitch)

            # 7. Move down to dropoff
            dropoff_pos_down = (10, 0, dropoff_z)
            self.move_to_position(dropoff_pos_down, pitch=pitch)

            # 8. Open gripper to drop
            self.open_gripper()

            # 9. Lift up after drop
            self.move_to_position((10, 0, 8), pitch=pitch)

            # 10. Reset base to center
            self.rotate_base(1500)

            # 11. Reset arm
            print("[Test] Resetting arm to rest position")
            self.AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1500)
            time.sleep(1)

        print("\n==== Varying Pitch Angles Test Done ====")

    def confirm_step(self, message):
        """
        Common method to confirm each step with the user.
        Returns True if user confirms, False otherwise.
        """
        print(f"\n[CONFIRM] {message}")
        response = input("Is this correct? (y/n): ").lower()
        return response == 'y'

    def identify_block(self):
        """
        Step 1: Identify block position and color.
        Returns (x, y, z, color) if successful, None if not.
        """
        print("\n[Step 1] Identifying block...")
        
        # Initialize camera
        from imx477.Camera import Camera  # Import from imx477 directory
        camera = Camera(resolution=(640, 480))
        
        # Print camera information
        print("\nCamera Information:")
        print(f"Resolution: 640x480")
        
        # Load LAB color data
        lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
        
        # Target colors to detect
        target_colors = ('red', 'green', 'blue')
        
        # Color mapping for drawing
        range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        
        try:
            while True:
                # Get frame from camera
                frame = camera.get_frame()
                if frame is None:
                    print("[ERROR] Failed to get camera frame")
                    return None
                
                # Convert frame to RGB like in main.py
                frame_rgb = frame[..., ::-1]
                    
                # Process frame
                frame_resize = cv2.resize(frame_rgb.copy(), (640, 480), interpolation=cv2.INTER_NEAREST)
                frame_lab = cv2.cvtColor(cv2.GaussianBlur(frame_resize, (3, 3), 3), cv2.COLOR_BGR2LAB)
                
                # Create display image
                display_img = cv2.resize(frame_rgb, (640, 480))
                
                # Find largest contour for each color
                color_area_max = None
                max_area = 0
                areaMaxContour_max = None
                
                for color in lab_data:
                    if color in target_colors:
                        mask = cv2.inRange(frame_lab, tuple(lab_data[color]['min']), tuple(lab_data[color]['max']))
                        closed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                        closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                        
                        # Find largest contour
                        for c in contours:
                            area = abs(cv2.contourArea(c))
                            if area > max_area and area > 300:  # Minimum area threshold
                                max_area = area
                                color_area_max = color
                                areaMaxContour_max = c
                
                # If we found a block
                if max_area > 500 and areaMaxContour_max is not None:
                    # Get center and radius of block
                    (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)
                    
                    # Print pixel coordinates and frame dimensions
                    print(f"\nBlock Detection:")
                    print(f"Pixel coordinates: ({int(center_x)}, {int(center_y)})")
                    print(f"Frame dimensions: {display_img.shape}")
                    print(f"Block radius in pixels: {int(radius)}")
                    
                    # Draw circle around detected block
                    cv2.circle(display_img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
                    
                    # Add text showing detected color and coordinates
                    cv2.putText(display_img, f"Detected: {color_area_max} at ({int(center_x)}, {int(center_y)})",
                               (10, display_img.shape[0] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color_area_max], 2)
                    
                    # Show the frame with detection
                    cv2.imshow('Block Detection', display_img)
                    cv2.waitKey(1)  # Update display
                    
                    # Convert pixel coordinates to real-world coordinates
                    # These values should be calibrated for your setup
                    x = 10  # Example: 10cm from base
                    y = 0   # Example: centered
                    z = 2   # Example: 2cm height
                    
                    print(f"[INFO] Found {color_area_max} block at ({x}, {y}, {z})")
                    cv2.destroyAllWindows()
                    return (x, y, z, color_area_max)
                
                # Show frame even when no block is detected
                cv2.putText(display_img, "No block detected",
                           (10, display_img.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb['black'], 2)
                cv2.putText(display_img, "Press 'q' to quit",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.imshow('Block Detection', display_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
                print("[INFO] No block detected")
                time.sleep(0.1)  # Small delay to prevent CPU overload
                
        finally:
            cv2.destroyAllWindows()
            return None

    def move_to_touch(self, block_pos):
        """
        Step 2: Move to just touch the block.
        Returns True if successful, False if not.
        """
        print("\n[Step 2] Moving to touch block...")
        x, y, z = block_pos[:3]
        # TODO: Implement movement to touch position
        return True

    def prepare_gripper(self, block_pos):
        """
        Step 3: Open gripper and orient for pickup.
        Returns True if successful, False if not.
        """
        print("\n[Step 3] Preparing gripper...")
        x, y, z = block_pos[:3]
        # TODO: Implement gripper preparation
        return True

    def grip_block(self, block_pos):
        """
        Step 4: Close gripper to hold block.
        Returns True if successful, False if not.
        """
        print("\n[Step 4] Gripping block...")
        x, y, z = block_pos[:3]
        # TODO: Implement block gripping
        return True

    def lift_block(self, block_pos):
        """
        Step 5: Lift block to safe height.
        Returns True if successful, False if not.
        """
        print("\n[Step 5] Lifting block...")
        x, y, z = block_pos[:3]
        # TODO: Implement block lifting
        return True

    def test_block_pickup_sequence(self):
        """
        Test the complete block pickup sequence with user confirmation at each step.
        """
        print("\n==== TEST: Block Pickup Sequence ====")
        
        # Step 1: Identify block
        block_pos = self.identify_block()
        if not block_pos:
            print("[ERROR] Failed to identify block")
            return
        if not self.confirm_step(f"Identified block at position {block_pos[:3]} with color {block_pos[3]}"):
            print("[ABORT] Block identification not confirmed")
            return

        # Step 2: Move to touch
        if not self.move_to_touch(block_pos):
            print("[ERROR] Failed to move to touch position")
            return
        if not self.confirm_step("Arm is in position to touch the block"):
            print("[ABORT] Touch position not confirmed")
            return

        # Step 3: Prepare gripper
        if not self.prepare_gripper(block_pos):
            print("[ERROR] Failed to prepare gripper")
            return
        if not self.confirm_step("Gripper is open and oriented correctly"):
            print("[ABORT] Gripper preparation not confirmed")
            return

        # Step 4: Grip block
        if not self.grip_block(block_pos):
            print("[ERROR] Failed to grip block")
            return
        if not self.confirm_step("Block is properly gripped"):
            print("[ABORT] Block grip not confirmed")
            return

        # Step 5: Lift block
        if not self.lift_block(block_pos):
            print("[ERROR] Failed to lift block")
            return
        if not self.confirm_step("Block is lifted to safe height"):
            print("[ABORT] Block lift not confirmed")
            return

        print("\n==== Block Pickup Sequence Complete ====")

if __name__ == '__main__':
    tester = PickupTest()
    
    # Run tests one at a time
    print("\nWhich test would you like to run?")
    print("1. Varying Pitch Angles")
    print("2. Block Pickup Sequence")
    
    choice = input("Enter test number (1-2): ")
    
    if choice == '1':
    tester.test_varying_pitch_angles()
    elif choice == '2':
        tester.test_block_pickup_sequence()
    else:
        print("Invalid choice")
