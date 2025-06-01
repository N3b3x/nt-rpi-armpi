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
            (x, 0, 10) for x in np.linspace(5, 19, num=6)
        ]
        
        # Define coordinates
        self.coordinates = {
            'capture': (0, 0, 0),  # Will be set from yaml
            'place': (12, 0, 0.5),  # Placing coordinate
        }
    
    def init_move(self):
        """Initialize arm to starting position."""
        self.board.pwm_servo_set_position(0.8, [[1, 1500]])
        self.AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1500)
    
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
        Execute the block pickup sequence.
        
        Args:
            x, y, z: Coordinates for pickup
        """
        # Open gripper
        self.board.pwm_servo_set_position(0.5, [[1, 1900]])
        time.sleep(0.8)
        
        # Move to pickup position
        pick_z = z - 2.75  # Go 2.75cm lower for pickup
        self.AK.setPitchRangeMoving((x, y, pick_z), -90, -90, 90, 1000)
        time.sleep(1)
        
        # Close gripper
        self.board.pwm_servo_set_position(0.5, [[1, 1500]])
        time.sleep(0.8)
        
        # Lift block
        self.AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
        time.sleep(1.5)
        
        # Adjust gripper
        self.board.pwm_servo_set_position(0.5, [[6, 1500]])
        time.sleep(1.5)
    
    def place_block(self):
        """Execute the block placement sequence."""
        # Move to placement position
        self.AK.setPitchRangeMoving((self.coordinates['place'][0], 
                                    self.coordinates['place'][1], 12), 
                                   -90, -90, 90, 800)
        time.sleep(0.8)
        
        # Calculate placement height based on stack position
        if self.number == 0:
            place_z = self.coordinates['place'][2] - 0.5
        elif self.number == 1:
            place_z = self.coordinates['place'][2] + 2
        else:
            place_z = self.coordinates['place'][2] + 2.5
            
        # Move to final placement height
        self.AK.setPitchRangeMoving((self.coordinates['place'][0], 
                                    self.coordinates['place'][1], 
                                    place_z), 
                                   -90, -90, 90, 800)
        time.sleep(0.8)
        
        # Open gripper
        self.board.pwm_servo_set_position(0.5, [[1, 1900]])
        time.sleep(0.8)
        
        # Return to home position
        self.AK.setPitchRangeMoving((6, 0, 18), -90, -90, 90, 1500)
        time.sleep(1.5)
        
        # Adjust gripper
        self.board.pwm_servo_set_position(0.5, [[6, 1500]])
        time.sleep(1.5)
        
        # Move to scan position
        self.AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 800)
        time.sleep(0.8)
        
        # Update stack counter
        self.number += 1
        if self.number == 3:
            self.number = 0
            self.board.set_buzzer(1900, 0.1, 0.9, 1)
            self.set_rgb('white')
            time.sleep(0.5)
    
    def scan_position(self, position):
        """
        Move to a scan position and wait for stabilization.
        
        Args:
            position: (x, y, z) tuple for scan position
        """
        self.AK.setPitchRangeMoving(position, -90, -90, 90, 800)
        time.sleep(0.7)  # Wait for arm to stabilize 