#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
from kinematics.arm_move_ik import *
from common.ros_robot_controller_sdk import Board

# 5.机械臂基础运动课程/第4课 机械臂三轴联动(5.Basic Motion Lesson/Lesson 4 Move on XYZ Axis)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
****************功能:逆运动学三轴联动例程(Function: Move on XYZ Axis in Inverse Kinematics)********************
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(Press 'Ctrl+C' to exit this program, if it fails, please try a few more times!)
----------------------------------------------------------
''')

# 实例化逆运动学库(Instantiate inverse kinematics library)
AK = ArmIK()
AK.board = Board()
 
if __name__ == "__main__":
    '''
    AK.setPitchRangeMoving(coordinate_data, alpha, alpha1, alpha2, movetime):
    给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置(Specify a coordinate 'coordinate_data' and a pitch angle 'pitch' ranging from 'alpha1' to 'alpha2' to find automatically the solution closest to the given pitch angle and rotate the robotic arm to the target position.)
    如果无解返回False,否则返回舵机角度、俯仰角、运行时间(If there is no solution, it returns to 'False'. Otherwise, it returns the servo angle, pitch angle, and running time.)
    坐标单位cm， 以元组形式传入，例如(0, 5, 10)(The coordinate is in the unit of cm, transmitted in tuple form. For example, (0, 5, 10))
    alpha: 为给定俯仰角(the given pitch angle)
    alpha1和alpha2: 为俯仰角的取值范围(the range of given pitch angle)
    movetime:为舵机转动时间，单位ms, 如果不给出时间，则自动计算(the rotation time of the servo in the unit of ms. If the time is not given, automatically calculate it.)    
    '''
    # 设置机械臂初始位置(x:0, y:6, z:18),运行时间:1500毫秒(set the initial position of the robotic arm to (x:0, y:6, z:18) and the runtime to 1500ms)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) 
    time.sleep(1.5) # 延时1.5秒(delay for 1.5s)
    


    AK.setPitchRangeMoving((5, 6, 18), 0,-90, 90, 1000)  # 设置机械臂X轴右移,运行时间:1000毫秒(set the robotic arm to move to the right along the X-axis with a runtime of 1000ms)
    time.sleep(1.2) # 延时1.2秒(delay for 1.2s)
    AK.setPitchRangeMoving((5, 13, 11), 0,-90, 90, 1000) #设置机械臂Y轴、Z轴同时移动，运行时间:1000毫秒(Set the robotic arm to move simultaneously along the Y and Z axes with a runtime of 1000ms.)
    time.sleep(1.2) # 延时1.2秒(delay for 1.2s)
    AK.setPitchRangeMoving((-5, 13, 11), 0,-90, 90, 1000) # 设置机械臂X轴右移,运行时间:1000毫秒(set the robotic arm to move to the right along the X-axis with a runtime of 1000ms)
    time.sleep(1.2) # 延时1.2秒(delay for 1.2s)
    AK.setPitchRangeMoving((-5, 6, 18), 0,-90, 90, 1000)  #设置机械臂Y轴、Z轴同时移动，运行时间:1000毫秒(Set the robotic arm to move simultaneously along the Y and Z axes with a runtime of 1000ms.)
    time.sleep(1.2) # 延时1.2秒(delay for 1.2s)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1000) # 设置机械臂X轴左移,运行时间:1000毫秒(set the robotic arm to move to the left along the X-axis with a runtime of 1000ms)
    time.sleep(1.2) # 延时1.2秒(delay for 1.2s)
    
    
    
    
        
    