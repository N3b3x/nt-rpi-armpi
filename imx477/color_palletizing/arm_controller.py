#!/usr/bin/python3
# coding=utf8
import time
import numpy as np
from kinematics.arm_move_ik import ArmIK
from common.ros_robot_controller_sdk import Board
import math

# Servo number constants (based on hardware mapping and provided image)
SERVO_GRIPPER = 1   # 1: Gripper (Claw)
SERVO_ELBOW   = 3   # 3: Elbow
SERVO_SHOULDER= 4   # 4: Shoulder
SERVO_LIFT    = 5   # 5: Base Lift/Rotate
SERVO_BASE    = 6   # 6: Base Rotation
# (If you have a wrist servo, add SERVO_WRIST = 2)

class ArmController:
    """
    Controls the robotic arm movements and operations.
    """
    def __init__(self):
        self.board = Board()
        self.AK = ArmIK()
        self.AK.board = self.board
        self.number = 0  # Counter for stacking
        
        # Define scan positions in polar coordinates (r, theta, z)
        # r: distance from base (cm)
        # theta: angle relative to forward direction (degrees)
        # z: height above table (cm)
        
        # Adjust scan radius based on angle:
        # - At 0° (forward), we can reach further
        # - At ±90° and beyond, we need shorter reach
        self.min_radius = 8
        self.max_radius_forward = 16  # Maximum reach when facing forward
        self.max_radius_side = 12    # Maximum reach at ±90°
        self.scan_heights = [18]     # Increased height for safety
        
        # Define coordinates
        self.coordinates = {
            'capture': (0, 0, 0),  # Will be set from yaml
            'place': (12, 0, 0.5),  # Placing coordinate
        }

        # Base rotation tracking
        self.current_base_angle = 0   # degrees, 0 = forward
        self.current_base_pos = 1500  # PWM signal for base servo
        self.base_rotation_angle = 0  # Track absolute base rotation

    def get_max_radius(self, base_angle):
        """Calculate maximum safe radius based on base angle."""
        # Convert angle to absolute value (symmetrical workspace)
        abs_angle = abs(base_angle)
        
        # Linear interpolation between forward and side max radius
        # As angle goes from 0° to 90°, max radius goes from forward to side
        if abs_angle <= 90:
            fraction = abs_angle / 90.0
            return self.max_radius_forward * (1 - fraction) + self.max_radius_side * fraction
        else:
            # Beyond ±90°, use an even shorter radius
            over_90 = (abs_angle - 90) / 30.0  # How far past 90° we are
            return max(self.min_radius, self.max_radius_side * (1 - over_90 * 0.25))

    def get_scan_positions(self, base_angle):
        """
        Get safe scan positions for this base angle.
        Adjust max radius based on angle to avoid overextending.
        """
        scan_positions = []
        max_r = self.get_max_radius(base_angle)
        
        # Use 3 radial positions between min and max radius
        radii = np.linspace(self.min_radius, max_r, num=3)
        
        # Convert base angle to radians
        base_rad = math.radians(base_angle)
        
        # Always use safe Z height (e.g., 18cm)
        safe_z = 18
        
        for r in radii:
            x = r * math.cos(base_rad)
            y = r * math.sin(base_rad)
            scan_positions.append((x, y, safe_z))
        
        return scan_positions

    def init_move(self):
        """Initialize arm to starting position (match Hiwonder sample)."""
        self.board.pwm_servo_set_position(0.3, [[SERVO_GRIPPER, 1500]])
        self.AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 1500)
    
    def set_rgb(self, color):
        """Set RGB LED color."""
        if color == "red":
            self.board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
        elif color == "green":
            self.board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
        elif color == "blue":
            self.board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
        else:
            self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
    
    def set_coordinates(self, x, y, z):
        """Set the capture coordinates."""
        self.coordinates['capture'] = (x, y, z)
    
    def pick_up_block(self, x, y, z):
        """
        Pick up block at (x, y, z).
        """
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1900]])  # Open
        time.sleep(0.8)

        print(f"[Arm] Lowering to pickup at ({x}, {y}, {z - 2.75})")
        self.AK.setPitchRangeMoving((x, y, z - 2.75), -90, -90, 90, 1000)
        time.sleep(1)

        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1500]])  # Close
        time.sleep(0.8)

        self.AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)  # Move up
        time.sleep(1.5)
    
    def place_block(self):
        """Execute the block placement sequence."""
        stacking_x, stacking_y, stacking_z = self.coordinates['place']
        block_height = 3
        self.AK.setPitchRangeMoving((stacking_x, stacking_y, 12), -90, -90, 90, 800)
        time.sleep(0.8)
        place_z = stacking_z + self.number * block_height - 2
        self.AK.setPitchRangeMoving((stacking_x, stacking_y, place_z), -90, -90, 90, 800)
        time.sleep(0.8)
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1900]])
        time.sleep(0.8)
        self.AK.setPitchRangeMoving((6, 0, 18), -90, -90, 90, 1500)
        time.sleep(1.5)
        self.AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 800)
        time.sleep(0.8)
        self.number = (self.number + 1) % 3
        if self.number == 0:
            self.board.set_buzzer(1900, 0.1, 0.9, 1)
            self.set_rgb('white')
            time.sleep(0.5)
    
    def scan_position(self, position, pitch_angle=-10):
        """
        Move to a scan position with a fixed pitch angle to avoid "lifting".
        """
        print(f"[Arm] Moving to scan position: {position} with pitch {pitch_angle}° (base angle {self.current_base_angle})")
        self.board.pwm_servo_set_position(0.5, [[SERVO_BASE, self.current_base_pos]])
        # Use fixed pitch angle and scan position
        self.AK.setPitchRangeMoving(position, pitch_angle, pitch_angle, pitch_angle, 500)
        time.sleep(0.25)
    
    def rotate_base(self, angle):
        """
        Rotate the base by a given angle (degrees). Positive for CW, negative for CCW.
        """
        self.current_base_angle += angle
        self.current_base_angle = max(-180, min(180, self.current_base_angle))  # Expand to ±180°
        units_per_degree = 1000 / 180
        self.current_base_pos = 1500 + int(self.current_base_angle * units_per_degree)
        self.current_base_pos = max(500, min(2500, self.current_base_pos))
        print(f"[Arm] Rotating base to servo pos {self.current_base_pos} for angle {self.current_base_angle}")
        self.board.pwm_servo_set_position(0.5, [[SERVO_BASE, self.current_base_pos]])
        self.base_rotation_angle = self.current_base_angle  # 💡 Always track absolute base rotation
        time.sleep(1)
