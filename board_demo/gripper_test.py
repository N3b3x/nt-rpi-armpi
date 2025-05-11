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
********功能:幻尔科技树莓派扩展板，夹爪舵机测试**********
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
        # Reset gripper to middle position
        board.pwm_servo_set_position(0.5, [[1, 1500]])  # Reset servo 1 to middle position
        time.sleep(0.5)
        print("Reset complete")
    except Exception as e:
        print(f"Error during reset: {e}")
    print('已关闭')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    try:
        # Test gripper servo (servo 1)
        print("\nTesting gripper servo (servo 1)...")
        
        # Set initial position (middle)
        print("Moving to middle position (1500)")
        board.pwm_servo_set_position(0.5, [[1, 1500]])
        time.sleep(1.5)

        # Move gripper to different positions
        for i in range(2):
            print("Moving to position 2000 (closed)")
            board.pwm_servo_set_position(1.0, [[1, 2000]])  # Close gripper
            time.sleep(1.2)
            
            print("Moving to position 1000 (open)")
            board.pwm_servo_set_position(1.0, [[1, 1000]])  # Open gripper
            time.sleep(1.2)
            
            print("Moving back to middle position")
            board.pwm_servo_set_position(1.0, [[1, 1500]])  # Middle position
            time.sleep(1.2)

        print("\nAll tests complete. Press Ctrl+C to exit.")
        while start:
            time.sleep(1)

    except Exception as e:
        print(f"Error in main loop: {e}")
        sys.exit(1) 