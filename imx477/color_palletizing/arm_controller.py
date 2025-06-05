#!/usr/bin/python3
# coding=utf8
import time
import numpy as np
from kinematics.arm_move_ik import ArmIK
from common.ros_robot_controller_sdk import Board

class ArmController:
    """
    Controls the robotic arm movements and operations.
    """
    def __init__(self):
        self.board = Board()
        self.AK = ArmIK()
        self.AK.board = self.board
        self.number = 0  # Counter for stacking
        
        # Define scan positions (side-to-side sweep along X axis at fixed Y, Z)
        self.scan_positions = [
            (x, 0, 10) for x in np.linspace(2, 22, num=8)
        ]
        
        # Define coordinates
        self.coordinates = {
            'capture': (0, 0, 0),  # Will be set from yaml
            'place': (12, 0, 0.5),  # Placing coordinate
        }

        self.current_base_pos = 1500  # 💡 Add this line!
    
    def init_move(self):
        """Initialize arm to starting position (match Hiwonder sample)."""
        self.board.pwm_servo_set_position(0.3, [[1, 1500]])
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
        Execute the block pickup sequence (match Hiwonder sample).
        Args:
            x, y, z: Coordinates for pickup
        """
        self.board.pwm_servo_set_position(0.5, [[1, 1900]])
        time.sleep(0.8)
        pick_z = z - 2.75
        self.AK.setPitchRangeMoving((x, y, pick_z), -90, -90, 90, 1000)
        time.sleep(1)
        self.board.pwm_servo_set_position(0.5, [[1, 1500]])
        time.sleep(0.8)
        self.AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
        time.sleep(1.5)
    
    def place_block(self):
        """Execute the block placement sequence (match Hiwonder sample)."""
        stacking_x, stacking_y, stacking_z = self.coordinates['place']
        block_height = 3
        self.AK.setPitchRangeMoving((stacking_x, stacking_y, 12), -90, -90, 90, 800)
        time.sleep(0.8)
        place_z = stacking_z + self.number * block_height - 2
        self.AK.setPitchRangeMoving((stacking_x, stacking_y, place_z), -90, -90, 90, 800)
        time.sleep(0.8)
        self.board.pwm_servo_set_position(0.5, [[1, 1900]])
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
    
    def scan_position(self, position):
        """
        Move to a scan position and wait for stabilization.
        
        Args:
            position: (x, y, z) tuple for scan position
        """
        print(f"[Arm] Moving to scan position: {position}")
        self.AK.setPitchRangeMoving(position, -90, -90, 90, 800)
        time.sleep(0.7)  # Wait for arm to stabilize 
    
    def rotate_base(self, angle):
        """Rotate the base by a given angle (degrees). Positive for CW, negative for CCW."""
        units_per_degree = 1000 / 180
        self.current_base_pos += int(angle * units_per_degree)
        self.current_base_pos = max(500, min(2500, self.current_base_pos))
        print(f"[Arm] Rotating base to servo pos {self.current_base_pos} for angle {angle}")
        self.board.pwm_servo_set_position(0.5, [[0, self.current_base_pos]])
        time.sleep(1)
