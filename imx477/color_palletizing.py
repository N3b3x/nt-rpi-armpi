#!/usr/bin/python3
# coding=utf8
import sys
import time
import math
import threading
import cv2
import numpy as np
import yaml
import os

# Module paths
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')

import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle
from Camera import Camera

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

lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
    print("✅ Loaded LAB data:", lab_data)

def getAreaMaxContour(contours):
    max_area = 0
    max_contour = None
    for c in contours:
        area = abs(cv2.contourArea(c))
        if area > max_area and area > 300:
            max_area = area
            max_contour = c
    return max_contour, max_area

__target_color = ('red', 'green', 'blue')
_stop = False
color_list = []
__isRunning = False
detect_color = 'None'
start_pick_up = False
number = 0
size = (640, 480)
draw_color = range_rgb["black"]
table_calib_file = 'table_height.yaml'  # Path to save table height
block_height = 3  # cm, as specified

table_z = None  # Will be set by calibration or loaded

# Add stacking location variables
stacking_X = 12  # Default stacking X (can be changed)
stacking_Y = 0   # Default stacking Y (can be changed)
stacking_Z = 0.5 # Default stacking Z (table height at stacking location)

stacking_calib_file = 'stacking_height.yaml'  # Path to save stacking Z

manual_mode = False
manual_x = None
manual_y = None
manual_z = None
manual_step = 1.0  # cm per keypress

def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

def reset():
    global _stop, color_list, detect_color, start_pick_up
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False

def init():
    global number
    number = 0
    print("ColorPalletizing Init")
    load_config()
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorPalletizing Start")

def stop():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorPalletizing Stop")

def run(img):
    global __isRunning, color_list, detect_color, draw_color, start_pick_up

    if not __isRunning:
        return img

    display_img = cv2.resize(img, size)
    if not start_pick_up:
        frame_resize = cv2.resize(img.copy(), size, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(cv2.GaussianBlur(frame_resize, (3, 3), 3), cv2.COLOR_BGR2LAB)

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0

        for i in lab_data:
            if i in __target_color:
                mask = cv2.inRange(frame_lab, tuple(lab_data[i]['min']), tuple(lab_data[i]['max']))
                cv2.imshow(f"Mask - {i}", mask)
                closed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                areaMaxContour, area = getAreaMaxContour(contours)
                if areaMaxContour is not None and area > max_area:
                    max_area = area
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour

        if max_area > 500:
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)
            cv2.circle(display_img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
            color_list.append({'red':1, 'green':2, 'blue':3}.get(color_area_max, 0))

            if len(color_list) == 3:
                color = int(round(np.mean(color_list)))
                color_list.clear()
                if color == 1:
                    detect_color = 'red'
                    draw_color = range_rgb["red"]
                    start_pick_up = True
                elif color == 2:
                    detect_color = 'green'
                    draw_color = range_rgb["green"]
                    start_pick_up = True
                elif color == 3:
                    detect_color = 'blue'
                    draw_color = range_rgb["blue"]
                    start_pick_up = True
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                detect_color = "None"
                draw_color = (0, 0, 0)

    # Always overlay detected color text, even during pickup
        cv2.putText(display_img, f"Detected Color: {detect_color}",
                    (10, display_img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    cv2.putText(display_img, "c-recalibrate | q-quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        return display_img

def initMove():
    board.pwm_servo_set_position(0.3, [[1, 1500]])
    AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 1500)

def move():
    global _stop, number, __isRunning, detect_color, start_pick_up
    x = Coordinates_data['X']
    y = Coordinates_data['Y']
    z = Coordinates_data['Z']
    block_height = 3  # cm
    stacking_x, stacking_y, stacking_z = 12, 0, 0.5  # HiWonder stacking location
    while True:
        if __isRunning and detect_color != 'None' and start_pick_up:
            set_rgb(detect_color)
            board.set_buzzer(1900, 0.1, 0.9, 1)
            board.pwm_servo_set_position(0.5, [[1, 1900]])
            time.sleep(0.8)
            if not __isRunning: continue

            # Pickup
            AK.setPitchRangeMoving((x, y, z - 2.75), -90, -90, 90, 1000)
            time.sleep(1)
            if not __isRunning: continue
            board.pwm_servo_set_position(0.5, [[1, 1500]])
            time.sleep(0.8)
            if not __isRunning: continue

            AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
            time.sleep(1.5)
            if not __isRunning: continue

            # Move above stacking location
            AK.setPitchRangeMoving((stacking_x, stacking_y, 12), -90, -90, 90, 800)
            time.sleep(0.8)
            if not __isRunning: continue

            # Place at stacking location, increment Z for each block, all 1 cm lower
            place_z = stacking_z + number * block_height - 2  # Drop all blocks 1 cm lower
            print(f"Placing block number {number} at X: {stacking_x}, Y: {stacking_y}, Z: {place_z}")
            AK.setPitchRangeMoving((stacking_x, stacking_y, place_z), -90, -90, 90, 800)
            time.sleep(0.8)
            board.pwm_servo_set_position(0.5, [[1, 1900]])
            time.sleep(0.8)

            AK.setPitchRangeMoving((6, 0, 18), -90, -90, 90, 1500)
            time.sleep(1.5)

            AK.setPitchRangeMoving((0, 8, 10), -90, -90, 90, 800)
            time.sleep(0.8)

            number = (number + 1) % 3
            if number == 0:
                board.set_buzzer(1900, 0.1, 0.9, 1)
                set_rgb('white')
                time.sleep(0.5)

            detect_color = 'None'
            start_pick_up = False
            set_rgb('None')
        else:
            time.sleep(0.01)

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

    init()
    start()
    camera = Camera(resolution=(640, 480))
    threading.Thread(target=move, daemon=True).start()

    print("\n=== Color Palletizing Controls ===")
    print("Press 'q' or ESC to quit\n")

    while True:
        frame = camera.get_frame()
        if frame is not None:
            frame_rgb = frame[..., ::-1]
            Frame = run(frame_rgb)

            cv2.imshow('IMX477 Color Palletizing', Frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("Quitting...")
                break
        else:
            time.sleep(0.01)

    stop()
    cv2.destroyAllWindows()
