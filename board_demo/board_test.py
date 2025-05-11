#!/usr/bin/python3
# coding=utf8
import sys
import time
import signal
import ros_robot_controller_sdk as rrc
from kinematics.arm_move_ik import ArmIK

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********功能:幻尔科技树莓派扩展板，基础功能测试**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

try:
    print("Initializing board...")
    board = rrc.Board()
    print("Board initialized successfully")
except Exception as e:
    print(f"Error initializing board: {e}")
    sys.exit(1)

start = True

def Stop(signum, frame):
    global start
    start = False
    print('关闭中...')
    try:
        # Turn off RGB (both LEDs)
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        # Reset servo
        AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1500)  # Reset to initial position
        time.sleep(0.5)
        print("Reset complete")
    except Exception as e:
        print(f"Error during reset: {e}")
    print('已关闭')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    try:
        # Test RGB - should work
        print("\nTesting RGB LED...")
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])  # Red
        time.sleep(1)
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])  # Green
        time.sleep(1)
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])  # Blue
        time.sleep(1)
        print("RGB test complete")

        # Test buzzer - should work
        print("\nTesting buzzer...")
        board.set_buzzer(1900, 0.1, 0.9, 1) # 1900Hz, 0.1s on, 0.9s off, 1 repeat
        time.sleep(2)
        board.set_buzzer(1000, 0.5, 0.5, 0) # 1000Hz, 0.5s on, 0.5s off, repeat
        time.sleep(3)
        board.set_buzzer(1000, 0.0, 0.0, 1) # Off
        print("Buzzer test complete")

        # Test servo with kinematics library
        print("\nTesting servo with kinematics library...")
        AK = ArmIK()
        AK.board = board

        # Set initial position
        AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1500)  # Initial position
        time.sleep(1.5)

        # Move up and down
        for i in range(2):
            AK.setPitchRangeMoving((0, 6, 22), 0, -90, 90, 1000)  # Move up
            time.sleep(1.2)
            AK.setPitchRangeMoving((0, 6, 18), 0, -90, 90, 1000)  # Move down
            time.sleep(1.2)

        print("\nAll tests complete. Press Ctrl+C to exit.")
        while start:
            time.sleep(1)

    except Exception as e:
        print(f"Error in main loop: {e}")
        sys.exit(1) 