#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import math
import threading
import numpy as np

# Add the path to the common module
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(os.path.join(project_root, 'armpi_mini_sdk', 'common_sdk'))

import common.misc as Misc
import common.yaml_handle as yaml_handle

# 6.AI Vision Games Lesson/Lesson 1 Color Recognition

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# read color threshold file
lab_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# set detected target color
__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# find the contour with the largest area
# parameter is the listing of contours to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # iterate through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # Only the contour with the area larger than 300 is considered valid to filter out disturbance.
                area_max_contour = c

    return area_max_contour, contour_area_max  # return the largest contour

# the closing angle of the gripper while grasping an object
servo1 = 1500

# initial position
def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90,1500)

   

# set the color of the RGB light on the expansion board to match the color to be tracked
def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

_stop = False
color_list = []
__isRunning = False
detect_color = 'None'
start_pick_up = False

# reset variables
def reset(): 
    global _stop
    global color_list
    global detect_color
    global start_pick_up
    
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False

# initialization
def init():
    print("ColorDetect Init")
    load_config()
    initMove()

# app starts the game
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app stops the game
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Stop")

# app exits the game
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Exit")

size = (320, 240)
# function for the robotic arm's movement
def move():
    global _stop
    global __isRunning
    global detect_color
    global start_pick_up
    

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # if the block is detected no moving for a period, the robotic arm starts to grip
                set_rgb(detect_color)
                board.set_buzzer(1900, 0.1, 0.9, 1)# set the buzzer to sound for 0.1s
                
                if detect_color == 'red' : # if red is detected, the robotic arm nods its head
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.2, [[3, 800]])
                        time.sleep(0.2)
                        board.pwm_servo_set_position(0.2, [[3, 600]])
                        time.sleep(0.2)
                        if not __isRunning:
                            continue

                    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 500)  # return to initial position
                    time.sleep(0.5)  
                    detect_color = 'None'
                    start_pick_up = False
                    set_rgb(detect_color)
                    
                else: # if green and blue are detected, it shakes its head
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.4, [[6, 1300]])
                        time.sleep(0.3)
                        board.pwm_servo_set_position(0.4, [[6, 1700]])
                        time.sleep(0.3)
                        if not __isRunning:
                            continue

                    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 500)  # return to initial position
                    time.sleep(0.5)
                    detect_color = 'None'
                    start_pick_up = False
                    set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                initMove()  # return to initial position
                time.sleep(1.5)               
            time.sleep(0.01)

# run a sub-thread
th = threading.Thread(target=move)
th.daemon = True
th.start()  

roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]
# image processing
def run(img):
    global roi
    global center_list
    global __isRunning
    global start_pick_up
    global last_x, last_y
    global detect_color, draw_color, color_list
    
    if not __isRunning:
        return img
    else:
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert the image to LAB space

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        if not start_pick_up:
            for i in lab_data:
                if i in __target_color:
                    frame_mask = cv2.inRange(frame_lab,
                                                 (lab_data[i]['min'][0],
                                                  lab_data[i]['min'][1],
                                                  lab_data[i]['min'][2]),
                                                 (lab_data[i]['max'][0],
                                                  lab_data[i]['max'][1],
                                                  lab_data[i]['max'][2]))  # perform bitwise operation on the original image and the mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # opening operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # closing operation
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find contours
                    areaMaxContour, area_max = getAreaMaxContour(contours)  # find the largest contour
                    if areaMaxContour is not None:
                        if area_max > max_area:  # find the maximum area
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 500:  # the maximum area has been found
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)  # get the minimum circumscribed circle
                center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
                center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))
                cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
                
                if not start_pick_up:
                    if color_area_max == 'red':  # red is the maximum
                        color = 1
                    elif color_area_max == 'green':  # green is the maximum
                        color = 2
                    elif color_area_max == 'blue':  # blue is the maximum
                        color = 3
                    else:
                        color = 0
                    color_list.append(color)
                    if len(color_list) == 3:  # multiple judgements
                        # get mean
                        color = int(round(np.mean(np.array(color_list))))
                        color_list = []
                        start_pick_up = True
                        if color == 1:
                            detect_color = 'red'
                            draw_color = range_rgb["red"]
                        elif color == 2:
                            detect_color = 'green'
                            draw_color = range_rgb["green"]
                        elif color == 3:
                            detect_color = 'blue'
                            draw_color = range_rgb["blue"]
                        else:
                            detect_color = 'None'
                            draw_color = range_rgb["black"]
            else:
                if not start_pick_up:
                    draw_color = (0, 0, 0)
                    detect_color = "None"
          
        
        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        
        return img

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    # instantiate the inverse kinematics library
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    #cap = cv2.VideoCapture(0)
    while True:
        ret,img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
