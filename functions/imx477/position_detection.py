#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
import common.misc as Misc
import common.yaml_handle as yaml_handle
from Camera import Camera

# IMX477 version of position detection

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
# Gripper closing angle
servo1 = 1500

# Initial position
def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 8, 10), -90,-90, 0,1500)

# Color RGB values
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Read color threshold file
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# Find contour with largest area
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                areaMaxContour = c

    return areaMaxContour, contour_area_max

size = (320, 240)
__isRunning = False
__target_color = ('red',)

# Image processing and tracking control
def run(img):
    global lab_data
    global __isRunning
   
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img
     
    area_max = 0
    areaMaxContour = 0
    frame_resize = cv2.resize(img_copy, size)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
    for i in __target_color:
        if i in lab_data:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[detect_color]['min'][0],
                                          lab_data[detect_color]['min'][1],
                                          lab_data[detect_color]['min'][2]),
                                         (lab_data[detect_color]['max'][0],
                                          lab_data[detect_color]['max'][1],
                                          lab_data[detect_color]['max'][2]))
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
    if area_max > 300:
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
        center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        print('Center_x: ',center_x,' Center_y: ',center_y)
        cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[detect_color], 2)
        cv2.putText(img, 'X:'+str(center_x)+' Y:'+str(center_y), (center_x-65, center_y+100 ), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[detect_color], 2)
                    
    return img

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    # Initialize inverse kinematics library
    AK = ArmIK()
    AK.board = board
    
    initMove()
    load_config()
    __isRunning = True
    __target_color = ('red',)
    
    # Use IMX477 camera
    camera = Camera()
    
    while True:
        img = camera.get_frame()
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, size)
            cv2.imshow('IMX477 Position Detection', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows() 