#!/usr/bin/python3
# coding=utf8
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
#sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/functions')
#sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/imx477')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
#sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/functions')
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

__target_color = ('red',)
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

_stop = False
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
def reset():
    global _stop, get_roi, __isRunning, detect_color, start_pick_up, __target_color, x_dis, z_dis
    x_dis = 1500
    y_dis = 6
    z_dis = 18
    _stop = False
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False

def init():
    print("ColorTracking Init")
    load_config()
    initMove()

def start():
    global __isRunning, __target_color
    reset()
    __isRunning = True
    __target_color = ('red',)  # Set target color after reset
    print("ColorTracking Start")

def stop():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")

def exit():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Exit")

rect = None
size = (640, 480)
display_size = (640, 480)
roi = ()
def run(img):
    global roi, rect, get_roi, __isRunning, detect_color, start_pick_up, img_h, img_w, x_dis, y_dis, z_dis
    img_h, img_w = img.shape[:2]
    img_small = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(img_small, (3, 3), 3)
    frame_rgb = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2RGB)
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
    center_y, center_x = size[1] // 2, size[0] // 2
    #print("BGR at center:", frame_gb[center_y, center_x])
    #print("LAB at center:", frame_lab[center_y, center_x])
    area_max = 0
    areaMaxContour = 0
    frame_mask = np.zeros((size[1], size[0]), dtype=np.uint8)
    for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(
                frame_lab,
                tuple(lab_data[detect_color]['min']),
                tuple(lab_data[detect_color]['max'])
            )
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
    # Prepare display image (resize original BGR to display size)
    display_img = cv2.resize(img, display_size)
    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)
    if area_max > 500:
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
        #print("Drawing color for detected:", detect_color)
        # Map coordinates to display size
        center_x_disp = int(center_x)
        center_y_disp = int(center_y)
        radius_disp = int(radius)
        rect = cv2.minAreaRect(areaMaxContour)
        box = np.intp(cv2.boxPoints(rect))
        box = np.intp(box * [display_size[0] / size[0], display_size[1] / size[1]])
        cv2.circle(display_img, (center_x_disp, center_y_disp), radius_disp, range_rgb[detect_color], 2)
        cv2.drawContours(display_img, [box], 0, range_rgb[detect_color], 2)
        # --- SERVO CONTROL CODE ---
        if __isRunning:
            x_pid.SetPoint = img_w / 2.0
            y_pid.SetPoint = img_h / 2.0
            x_pid.update(center_x)
            y_pid.update(center_y)
            dx = x_pid.output
            dy = y_pid.output
            x_dis += dx
            y_dis += dy
            # Clamp x_dis and y_dis to safe limits
            x_dis = 500 if x_dis < 500 else x_dis
            x_dis = 2500 if x_dis > 2500 else x_dis
            y_dis = 5.00 if y_dis < 5.00 else y_dis
            y_dis = 10.00 if y_dis > 10.00 else y_dis
            # Use AK.setPitchRange to get servo angles
            target = AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), -90, 90)
            if target:
                servo_data = target[0]
                # Only move if the change is significant
                if abs(dx) > 2 or abs(dy) > 0.1:
                    board.pwm_servo_set_position(0.02, [[3, servo_data['servo3']],
                                                        [4, servo_data['servo4']],
                                                        [5, servo_data['servo5']],
                                                        [6, int(x_dis)]])
                    time.sleep(0.05)
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
    setTargetColor(('blue',))
    camera = Camera(resolution=(640, 480))  # Use full IMX477 resolution
    while True:
        frame = camera.get_frame()
        if frame is not None:
            Frame = run(frame)
            cv2.imshow('IMX477 Color Tracking', Frame)
            # No mouse callback needed
            key = cv2.waitKey(1)
            if key == 27:
                break
            elif key == ord('r'):
                setTargetColor(('red',))
                print("Switched to red")
            elif key == ord('b'):
                setTargetColor(('blue',))
                print("Switched to blue")
            elif key == ord('g'):
                setTargetColor(('green',))
                print("Switched to green")
        else:
            time.sleep(0.01)
    stop()
    cv2.destroyAllWindows() 