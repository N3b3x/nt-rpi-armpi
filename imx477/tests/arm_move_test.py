#!/usr/bin/python3
# pickup_test.py
# Testing pickup sequence with varying pitch angles.

import sys
import time
import math
import logging
from common.ros_robot_controller_sdk import Board

# Logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Path setup for local my_kinematics module
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')
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

    def rotate_base(self, pulse):
        print(f"[Test] Rotating base servo6 to pulse {pulse}")
        self.board.pwm_servo_set_position(0.5, [[SERVO_BASE, pulse]])
        time.sleep(0.8)

    def move_to_position(self, pos, pitch=-90, movetime=1000):
        result = self.AK.setPitchRangeMoving(pos, pitch, pitch, 0, movetime)
        if not result:
            print(f"[WARNING] Could not reach position: {pos} with pitch {pitch}")
        else:
            time.sleep(result[2] / 1000)

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

if __name__ == '__main__':
    tester = PickupTest()
    tester.test_varying_pitch_angles()
