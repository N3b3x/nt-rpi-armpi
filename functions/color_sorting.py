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


# 6.AI Vision Games Lesson/Lesson 2 Color Sorting

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# read gripping coordinate
Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# read color threshold file
lab_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

__target_color = ('red', 'green', 'blue')
# set target color
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# find the contour with the largest area
# parameter is the listing of contours to be compared
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : # iterate through all contours
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
    AK.setPitchRangeMoving((0, 8, 10), 0,-90, 90,1500)


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
    global __target_color

    _stop = False
    color_list = []
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False

# initialization
def init():
    print("ColorSorting Init")
    load_config()
    initMove()

# app starts the game
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

# app stops the game
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorSorting Stop")

# app exits the game
def exit():
    global _stop
    global __isRunning
    _stop = True
    set_rgb('None')
    __isRunning = False
    print("ColorSorting Exit")



size = (320, 240)
unreachable = False 
# function for the robotic arm's movement
def move():
    global rect
    global _stop
    global unreachable
    global __isRunning
    global detect_color
    global start_pick_up
    
    x = Coordinates_data['X']
    y = Coordinates_data['Y']
    z = Coordinates_data['Z']
    # placing coordinate
    coordinate = {
        'red':   (-11, 16, 2),
        'green': (-11, 8,  1),
        'blue':  (-11, 0, 0.5),
        'capture': (x, y, z)
    }
    
    while True:
        if __isRunning:        
            if detect_color != 'None' and start_pick_up: 
                # move to the target position with the height of 6cm, and determine if the specified position is reached based on the returned outcome
                # if the runtime parameter is not given, it calculates automatically and returns based on the outcome
                set_rgb(detect_color) # set the RGB light on the expansion board to light corresponding color
                board.set_buzzer(1900, 0.1, 0.9, 1)# set the buzzer to sound for 0.1s
                
                board.pwm_servo_set_position(0.5, [[1, 2000]])# open the gripper
                time.sleep(0.5)
                if not __isRunning: # detect if the game is stopped
                    continue
                AK.setPitchRangeMoving((coordinate['capture']), -90,-90, 0, 1000) # robotic arm runs to the gripping position
                print("(coordinate['capture']):",(coordinate['capture']))
                time.sleep(1) # delay for 1s
                if not __isRunning:
                    continue
                board.pwm_servo_set_position(0.5, [[1, 1500]])# close the gripper
                time.sleep(0.5)
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) # uplift the robotic arm
                time.sleep(1.5)
                if not __isRunning:
                    continue
                # first rotate the robotic arm, according to different colors, rotate to different positions
                if detect_color == 'red': 
                    board.pwm_servo_set_position(0.5, [[6, 1900]])
                    time.sleep(0.5)
                elif detect_color == 'green':
                    board.pwm_servo_set_position(0.8, [[6, 2100]])
                    time.sleep(0.8)
                elif detect_color == 'blue':
                    board.pwm_servo_set_position(1.5, [[6, 2500]])
                    time.sleep(1.5)
                if not __isRunning:
                    continue
                result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 8), -90, -90, 0, 1000) # robotic arm runs to the gripping position above
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) # if the specified position can be reached, it obtains the runtime
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000) # robotic arm runs to the gripping position
                time.sleep(0.5)
                if not __isRunning:
                    continue
                board.pwm_servo_set_position(0.5, [[1, 1800]]) # open the gripper, put down the block
                time.sleep(0.5)
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 8), -90, -90, 0, 800) # uplift the robotic arm
                time.sleep(0.8)
                if not __isRunning:
                    continue
                # robotic arm resets in stages
                board.pwm_servo_set_position(1.2, [[1, 1500],[3, 515],[4, 2170],[5, 945]])
                time.sleep(1.2)
                if detect_color == 'red':
                    board.pwm_servo_set_position(0.5, [[6, 1500]])
                    time.sleep(0.5)
                elif detect_color == 'green':
                    board.pwm_servo_set_position(0.8, [[6, 1500]])
                    time.sleep(0.8)
                elif detect_color == 'blue':
                    board.pwm_servo_set_position(1.5, [[6, 1500]])
                    time.sleep(1.5)
                AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1000)
                time.sleep(1)
                start_pick_up = False
                detect_color = 'None'
                set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)
          
# run sub-thread
th = threading.Thread(target=move)
th.daemon = True
th.start()      


draw_color = range_rgb["black"]
# image processing
def run(img):
    global unreachable
    global __isRunning
    global start_pick_up
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
                    closed[0:80, :] = 0
                    closed[:, 0:120] = 0
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
                        if color == 1:
                            detect_color = 'red'
                            draw_color = range_rgb["red"]
                            start_pick_up = True
                        elif color == 2:
                            detect_color = 'green'
                            draw_color = range_rgb["green"]
                            start_pick_up = True
                        elif color == 3:
                            start_pick_up = True
                            detect_color = 'blue'
                            draw_color = range_rgb["blue"]
                        else:
                            start_pick_up = False
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
    __target_color = ('red', 'green', 'blue')
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
