#!/usr/bin/python3
# coding=utf8
import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********Function: Hiwonder Raspberry Pi Expansion Board, Control Multiple PWM Servos**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program, if it fails please try multiple times!
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
    print('Closing...')
    try:
        # Reset servo 2 to middle position
        print("Attempting to reset servo position...")
        board.pwm_servo_set_position(0.5, [[6, 1500]])
        time.sleep(0.5)
        print("Reset complete")
    except Exception as e:
        print(f"Error during reset: {e}")
    print('Closed')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    print("Testing servo 2 with debug information...")
    try:
        # First move to middle position
        print("Moving to initial position...")
        board.pwm_servo_set_position(0.5, [[6, 1500]])
        time.sleep(1)
        print("Initial position set")
        
        while start:
            try:
                # Test just two positions with longer delays
                print("\nMoving to position 1000")
                board.pwm_servo_set_position(1.0, [[6, 1000]])
                time.sleep(2)
                
                print("Moving to position 2000")
                board.pwm_servo_set_position(1.0, [[6, 2000]])
                time.sleep(2)
                
            except Exception as e:
                print(f"Error during movement: {e}")
                time.sleep(1)
                
    except Exception as e:
        print(f"Error in main loop: {e}")
        sys.exit(1)
    
    
        
