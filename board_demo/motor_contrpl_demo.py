#!/usr/bin/python3
# coding=utf8
import sys
import time
import signal
import ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********Function: Hiwonder Raspberry Pi Expansion Board, Control DC Motors**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program, if it fails please try multiple times!
----------------------------------------------------------
''')
board = rrc.Board()

start = True
# Pre-shutdown processing
def Stop(signum, frame):
    global start

    start = False
    print('Closing...')
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])  # Turn off all motors

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    
    while True:
        board.set_motor_duty([[1, 35]])  # Set motor 1 speed to 35
        time.sleep(2)
        board.set_motor_duty([[1, 90]])  # Set motor 1 speed to 90
        time.sleep(1) 
        board.set_motor_duty([[1, 0]])   
        time.sleep(6)
        if not start:
            board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])  # Turn off all motors
            print('Closed')
            break
    
    
        
