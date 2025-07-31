#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
from kinematics.arm_move_ik import *
from common.ros_robot_controller_sdk import Board

# 5.Basic Motion Lesson/Lesson 4 Move on XYZ Axis

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
****************Function: Move on XYZ Axis in Inverse Kinematics********************
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
Tips:
 * Press 'Ctrl+C' to exit this program, if it fails, please try a few more times!
----------------------------------------------------------
''')

# Instantiate inverse kinematics library
AK = ArmIK()
AK.board = Board()
 
if __name__ == "__main__":
    '''
    AK.setPitchRangeMoving(coordinate_data, alpha, alpha1, alpha2, movetime):
    Specify a coordinate 'coordinate_data' and a pitch angle 'pitch' ranging from 'alpha1' to 'alpha2' to find automatically the solution closest to the given pitch angle and rotate the robotic arm to the target position.
    If there is no solution, it returns to 'False'. Otherwise, it returns the servo angle, pitch angle, and running time.
    The coordinate is in the unit of cm, transmitted in tuple form. For example, (0, 5, 10)
    alpha: the given pitch angle
    alpha1 and alpha2: the range of given pitch angle
    movetime: the rotation time of the servo in the unit of ms. If the time is not given, automatically calculate it.    
    '''
    # set the initial position of the robotic arm to (x:0, y:6, z:18) and the runtime to 1500ms
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) 
    time.sleep(1.5) # delay for 1.5s
    


    AK.setPitchRangeMoving((5, 6, 18), 0,-90, 90, 1000)  # set the robotic arm to move to the right along the X-axis with a runtime of 1000ms
    time.sleep(1.2) # delay for 1.2s
    AK.setPitchRangeMoving((5, 13, 11), 0,-90, 90, 1000) # Set the robotic arm to move simultaneously along the Y and Z axes with a runtime of 1000ms.
    time.sleep(1.2) # delay for 1.2s
    AK.setPitchRangeMoving((-5, 13, 11), 0,-90, 90, 1000) # set the robotic arm to move to the right along the X-axis with a runtime of 1000ms
    time.sleep(1.2) # delay for 1.2s
    AK.setPitchRangeMoving((-5, 6, 18), 0,-90, 90, 1000)  # Set the robotic arm to move simultaneously along the Y and Z axes with a runtime of 1000ms.
    time.sleep(1.2) # delay for 1.2s
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1000) # set the robotic arm to move to the left along the X-axis with a runtime of 1000ms
    time.sleep(1.2) # delay for 1.2s
    
    
    
    
        
    