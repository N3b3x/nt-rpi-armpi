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

# Read picking coordinates
Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

range_rgb = {
    'red': (0, 0, 255),    # BGR for red
    'blue': (255, 0, 0),   # BGR for blue
    'green': (0, 255, 0),  # BGR for green
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Read color threshold file
lab_data = None
def load_config():
    global lab_data
    print("yaml_handle loaded from:", yaml_handle.__file__)
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)

# Set target colors
__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color
    print("COLOR", target_color)
    __target_color = target_color
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

# Set RGB light color
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
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False

# Reset variables
def reset():
    global _stop
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    
    _stop = False
    color_list = []
    get_roi = False
    detect_color = 'None'
    start_pick_up = False

# Initialize
def init():
    global number
    number = 0
    print("ColorPalletizing Init")
    load_config()
    initMove()

# Start the game
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorPalletizing Start")

# Stop the game
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorPalletizing Stop")

# Exit the game
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorPalletizing Exit")

roi = ()
center_list = []
draw_color = range_rgb["black"]
size = (640, 480)

# Image processing
def run(img):
    global roi
    global get_roi
    global center_list
    global __isRunning
    global start_pick_up
    global detect_color, draw_color, color_list
    
    if not __isRunning:
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    # Always prepare display_img
    display_img = cv2.resize(img, size)
    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_rgb = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2RGB)
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    
    if not start_pick_up:
        for i in lab_data:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                       tuple(lab_data[i]['min']),
                                       tuple(lab_data[i]['max']))
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                closed[0:80, :] = 0
                closed[:, 0:120] = 0
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = getAreaMaxContour(contours)
                
                if areaMaxContour is not None:
                    if area_max > max_area:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour

        if max_area > 500:
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)
            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            cv2.circle(display_img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
            
            if not start_pick_up:
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
    
    cv2.putText(display_img, "Color: " + detect_color, (10, display_img.shape[0] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return display_img

servo1 = 1500

def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 1500)

def move():
    global _stop, get_roi, number, __isRunning, detect_color, start_pick_up

    # Example coordinates, adjust as needed
    x = Coordinates_data['X']
    y = Coordinates_data['Y']
    z = Coordinates_data['Z']

    coordinate = {
        'capture': (x, y, z),       # Gripping coordinate
        'place': (12, 0, 0.5),      # Placing coordinate
    }

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:
                set_rgb(detect_color)
                board.set_buzzer(1900, 0.1, 0.9, 1)
                board.pwm_servo_set_position(0.5, [[1, 1900]])  # Open gripper - less extreme
                time.sleep(0.8)  # Give more time to open
                if not __isRunning: continue
                
                # Adjust pick height to go 2.75cm lower
                pick_z = z - 2.75  # Go 2.75cm lower for pickup
                AK.setPitchRangeMoving((x, y, pick_z), -90, -90, 90, 1000)
                time.sleep(1)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[1, 1500]])  # Close gripper - tighter
                time.sleep(0.8)  # Give more time to close
                if not __isRunning: continue
                
                # Move to intermediate position with pitch -90
                AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
                time.sleep(1.5)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])
                time.sleep(1.5)
                if not __isRunning: continue
                
                # Move to place position
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], 12), -90, -90, 90, 800)
                time.sleep(0.8)
                if not __isRunning: continue
                
                # Adjusted stacking heights
                if number == 0:  # First block - 0.5cm lower
                    place_z = coordinate['place'][2] - 0.5
                elif number == 1:  # Second block - 0.5cm lower
                    place_z = coordinate['place'][2] + 2  # 3 - 1 = 2
                else:  # Third block - unchanged
                    place_z = coordinate['place'][2] + 2.5  # Keep as is
                
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], place_z), -90, -90, 90, 800)
                time.sleep(0.8)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[1, 1900]])  # Open gripper - less extreme
                time.sleep(0.8)  # Give more time to open
                if not __isRunning: continue
                
                # Move to intermediate position with pitch -90
                AK.setPitchRangeMoving((6, 0, 18), -90, -90, 90, 1500)
                time.sleep(1.5)
                if not __isRunning: continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])
                time.sleep(1.5)
                if not __isRunning: continue
                
                # Return to initial position with pitch -90
                AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 800)
                time.sleep(0.8)

                number += 1
                if number == 3:
                    number = 0
                    board.set_buzzer(1900, 0.1, 0.9, 1)
                    set_rgb('white')
                    time.sleep(0.5)

                if not __isRunning: continue
                # Reset state variables
                detect_color = 'None'
                get_roi = False
                start_pick_up = False
                set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                initMove()
            time.sleep(0.01)

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
    
    # Start the move thread
    th = threading.Thread(target=move)
    th.daemon = True
    th.start()
    
    while True:
        frame = camera.get_frame()
        if frame is not None:
            Frame = run(frame)
            cv2.imshow('IMX477 Color Palletizing', Frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
        else:
            time.sleep(0.01)
    
    stop()
    cv2.destroyAllWindows() 