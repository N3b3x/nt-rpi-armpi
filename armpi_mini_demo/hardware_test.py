#!/usr/bin/python3
# coding=utf8
import sys
import time
import common.yaml_handle as yaml_handle
from common.ros_robot_controller_sdk import Board

board = Board()
deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********************PWM舵机测试(PWM Servo Testing)************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 *Press 'Ctrl+C' to exit this program, if it fails, please try a few more times!)
----------------------------------------------------------
''')
board.pwm_servo_set_position(0.3, [[1, 1800]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1500+ deviation_data['1']]]) 
time.sleep(1.5)

board.pwm_servo_set_position(0.3, [[3, 900]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 695]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 500]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 695+ deviation_data['3']]]) 
time.sleep(1.5)

board.pwm_servo_set_position(0.3, [[4, 2215]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 2415]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 2215]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 2415+ deviation_data['4']]]) 
time.sleep(1.5)

board.pwm_servo_set_position(0.3, [[5, 580]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 780]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 980]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 780+ deviation_data['5']]]) 
time.sleep(1.5)

board.pwm_servo_set_position(0.3, [[6, 1800]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1500+ deviation_data['6']]]) 
time.sleep(1.5)
