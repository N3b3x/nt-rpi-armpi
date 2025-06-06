#!/usr/bin/python3
# pickup_test.py
# Test script to validate robot arm pickup depths and angles.

import sys
import time
import math

sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')

from kinematics.arm_move_ik import ArmIK
from common.ros_robot_controller_sdk import Board

# Constants
SERVO_GRIPPER = 1
SERVO_BASE = 6

class PickupTest:
    def __init__(self):
        self.board = Board()
        self.AK = ArmIK()
        self.AK.board = self.board

        # Starting base angle
        self.base_angle = 0
        self.base_servo_pos = 1500

    def rotate_base(self, angle):
        """
        Rotate the base to a new absolute angle.
        """
        self.base_angle = max(-180, min(180, angle))
        units_per_degree = 1000 / 180
        self.base_servo_pos = 1500 + int(self.base_angle * units_per_degree)
        self.base_servo_pos = max(500, min(2500, self.base_servo_pos))

        print(f"[Test] Rotating base to angle {self.base_angle}° (servo pos {self.base_servo_pos})")
        self.board.pwm_servo_set_position(0.5, [[SERVO_BASE, self.base_servo_pos]])
        time.sleep(1)

    def open_gripper(self):
        print("[Test] Opening gripper")
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1900]])
        time.sleep(0.5)

    def close_gripper(self):
        print("[Test] Closing gripper")
        self.board.pwm_servo_set_position(0.5, [[SERVO_GRIPPER, 1500]])
        time.sleep(0.5)

    def test_pickup_depths(self):
        """
        Test pickup at different Z depths (2, 5, 8 cm).
        """
        x, y = 8, 0  # Position directly in front of arm
        depths = [2, 5, 8]
        pitch = -90  # 💡 Use steep angle to ensure downward reach

        self.rotate_base(0)
        self.open_gripper()

        for z in depths:
            print(f"[Test] Testing pickup at z={z} cm with pitch {pitch}°")
            self.AK.setPitchRangeMoving((x, y, z), pitch, pitch, 90, 2000)
            time.sleep(2)
            self.close_gripper()
            time.sleep(1)
            self.open_gripper()
            time.sleep(1)

    def test_pitch_angles(self):
        """
        Test different pitch angles at a fixed (x, y, z).
        """
        x, y, z = 8, 0, 5  # Typical pickup height
        pitches = [-45, -60, -90]

        self.rotate_base(0)
        self.open_gripper()

        for pitch in pitches:
            print(f"[Test] Testing pitch angle {pitch}° at z={z}")
            self.AK.setPitchRangeMoving((x, y, z), pitch, pitch, 90, 2000)
            time.sleep(2)
            self.close_gripper()
            time.sleep(1)
            self.open_gripper()
            time.sleep(1)

    def test_pickup_at_detected_position(self, x, y, z=5):
        """
        Test pickup at the last detected (x, y) position from scanning.
        """
        pitch = -60
        self.rotate_base(0)
        self.open_gripper()
        print(f"[Test] Moving to detected pickup at ({x}, {y}, {z}) with pitch {pitch}°")
        self.AK.setPitchRangeMoving((x, y, z), pitch, pitch, 90, 2000)
        time.sleep(2)
        self.close_gripper()
        time.sleep(1)
        self.open_gripper()
        time.sleep(1)

if __name__ == '__main__':
    tester = PickupTest()
    
    print("\n==== TEST 1: Pickup Depths ====")
    tester.test_pickup_depths()

    print("\n==== TEST 2: Pitch Angles ====")
    tester.test_pitch_angles()

    # To test at a detected position (example coordinates)
    # print("\n==== TEST 3: Pickup at Detected Position ====")
    # tester.test_pickup_at_detected_position(-2, -7, 5)

    print("\n==== Tests Complete ====")
