#!/usr/bin/python3
# coding=utf8
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
import cv2
import time
import math
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle
from Camera import Camera

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),    # BGR for red
    'blue': (255, 0, 0),   # BGR for blue
    'green': (0, 255, 0),  # BGR for green
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data
    print("yaml_handle loaded from:", yaml_handle.__file__)
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)

target_color = ('red', 'green', 'blue')
def setTargetColor(target_color_):
    global target_color
    target_color = target_color_
    return (True, ())

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

servo1 = 1500
x_dis = 1500
y_dis = 6
Z_DIS = 18
z_dis = Z_DIS
x_pid = PID.PID(P=0.26, I=0.05, D=0.008)
y_pid = PID.PID(P=0.012, I=0, D=0.000)
z_pid = PID.PID(P=0.003, I=0, D=0)

def initMove():
    board.pwm_servo_set_position(0.8, [[1, servo1]])
    AK.setPitchRangeMoving((0, y_dis, z_dis), 0,-90, 90, 1500)

def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

__isRunning = False
detect_color = 'None'
size = (640, 480)
interval_time = 0
draw_color = range_rgb["black"]
color_list = []

def init():
    print("ColorWarning Init")
    load_config()
    initMove()

def start():
    global __isRunning
    __isRunning = True
    print("ColorWarning Start")

def stop():
    global __isRunning
    __isRunning = False
    set_rgb('None')
    print("ColorWarning Stop")

def run(img):
    global interval_time, __isRunning, color_list, detect_color, draw_color
    
    if not __isRunning:
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_rgb = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2RGB)
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
    
    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    
    for i in lab_data:
        if i in target_color:
            frame_mask = cv2.inRange(frame_lab,
                                   tuple(lab_data[i]['min']),
                                   tuple(lab_data[i]['max']))
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
            
            if areaMaxContour is not None:
                if area_max > max_area:
                    max_area = area_max
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour
    
    # Prepare display image
    display_img = cv2.resize(img, size)
    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)
    
    if max_area > 2500:
        rect = cv2.minAreaRect(areaMaxContour_max)
        box = np.intp(cv2.boxPoints(rect))
        
        cv2.drawContours(display_img, [box], -1, range_rgb[color_area_max], 2)
        
        if color_area_max == 'red':
            color = 1
        elif color_area_max == 'green':
            color = 2
        elif color_area_max == 'blue':
            color = 3
        else:
            color = 0
            
        color_list.append(color)
        if len(color_list) == 3:
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            
            if color == 1:
                if time.time() > interval_time:
                    interval_time = time.time() + 3
                    for i in range(1):
                        board.set_buzzer(1900, 0.1, 0.9, 1)
                        time.sleep(0.1)
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
        draw_color = (0, 0, 0)
        detect_color = "None"
    
    set_rgb(detect_color)
    cv2.putText(display_img, "Color: " + detect_color, (10, display_img.shape[0] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return display_img

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    print("Board initialized:", board)
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    camera = Camera(resolution=(640, 480))  # Use full IMX477 resolution
    
    while True:
        frame = camera.get_frame()
        if frame is not None:
            Frame = run(frame)
            cv2.imshow('IMX477 Color Warning', Frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
        else:
            time.sleep(0.01)
    
    stop()
    cv2.destroyAllWindows() 