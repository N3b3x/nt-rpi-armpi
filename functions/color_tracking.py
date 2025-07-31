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

import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle

# 6.AI视觉学习课程/第4课 目标追踪(6.AI Vision Games Lesson/Lesson 2 Color Tracking)

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

# 读取颜色阈值文件(read color threshold file)
lab_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

__target_color = ('red',)
# 设置检测颜色(set target color)
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# 找出面积最大的轮廓(find the contour with the largest area)
# 参数为要比较的轮廓的列表(parameter is the listing of contours to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(Only the contour with the area larger than 300, which is greater than 300, is considered valid to filter out disturbance.)
                areaMaxContour = c

    return areaMaxContour, contour_area_max  # 返回最大的轮廓(return the largest contour)

# 夹持器夹取时闭合的角度(the closing angle of the gripper while grasping an object)
servo1 = 1500
x_dis = 1500
y_dis = 6
Z_DIS = 18
z_dis = Z_DIS
x_pid = PID.PID(P=0.26, I=0.05, D=0.008)  # pid初始化(pid initialization)
y_pid = PID.PID(P=0.012, I=0, D=0.000)
z_pid = PID.PID(P=0.003, I=0, D=0)

# 初始位置(initial position)
def initMove():
    board.pwm_servo_set_position(0.8, [[1, servo1]])
    AK.setPitchRangeMoving((0, y_dis, z_dis), 0,-90, 90, 1500)

#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致(set the color of the RGB light on the expansion board to match the color to be tracked)
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
# 变量重置(reset variables)
def reset():
    global _stop
    global get_roi
    global __isRunning
    global detect_color
    global start_pick_up
    global __target_color
    global x_dis,z_dis
    
    x_dis = 1500
    y_dis = 6
    z_dis = 18
    _stop = False
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False

# app初始化调用(call the initialization of the app)
def init():
    print("ColorTracking Init")
    load_config()
    initMove()

# app开始玩法调用(the app starts the game calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app停止玩法调用(the app stops the game calling)
def stop():
    global _stop 
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")

# app退出玩法调用(the app exits the game calling)
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Exit")

rect = None
size = (320, 240)


roi = ()
# 图像处理及追踪控制(image processing and tracking control)
def run(img):
    global roi
    global rect
    global get_roi
    global __isRunning
    global detect_color
    global start_pick_up
    global img_h, img_w
    global x_dis, y_dis, z_dis
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img
     
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止(if an object is detected in a certain area, keep detecting the area until no object is detected)
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)    
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    
    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:
        for i in lab_data:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[detect_color]['min'][0],
                                              lab_data[detect_color]['min'][1],
                                              lab_data[detect_color]['min'][2]),
                                             (lab_data[detect_color]['max'][0],
                                              lab_data[detect_color]['max'][1],
                                              lab_data[detect_color]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation on the original image and the mask)
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算(opening operation)
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算(closing operation)
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
                areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find the largest contour)
        if area_max > 500:  # 有找到最大面积(the largest area has been found)
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆(get the minimum circumscribed circle)
            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))     
            
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.intp(cv2.boxPoints(rect))
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[detect_color], 2)
            
            
            if __isRunning:
                # 通过PID算法进行X轴追踪,根据目标的画面像素坐标与画面中心坐标比较进行追踪(Track the X-axis using PID algorithm, based on the comparison of the target's pixel coordinates with the center coordinates of the image.)
                x_pid.SetPoint = img_w / 2.0  # Set the target to the center of the image width
                x_pid.update(center_x)        # Update the PID controller with the detected object's x position
                dx = x_pid.output             # Get the PID output (how much to move)
                x_dis += int(dx)              # Adjust the arm's x position
                # Clamp x_dis to safe limits
                x_dis = 500 if x_dis < 500 else x_dis
                x_dis = 2500 if x_dis > 2500 else x_dis
                    
                # 通过PID算法进行Y轴追踪,根据目标的画面像素面积和设定值比较进行追踪(Track the Y-axis using PID algorithm, based on the comparison of the target image's pixel area with the set value.)
                y_pid.SetPoint = 80           # Target radius (size of the detected object in pixels)
                if abs(radius - 80) < 10:
                    radius = 80
                else:
                    if radius > 80:
                        radius = radius * 0.85
                y_pid.update(radius)          # Update PID with current radius
                dy = y_pid.output             # Get PID output
                y_dis += dy                   # Adjust arm's y position
                # Clamp y_dis to safe limits
                y_dis = 5.00 if y_dis < 5.00 else y_dis
                y_dis = 10.00 if y_dis > 10.00 else y_dis
                
                # 通过PID算法进行Z轴追踪,根据目标的画面像素坐标与画面中心坐标比较进行追踪(Track the Z-axis using PID algorithm, based on the comparison of the target's pixel coordinates with the center coordinates of the image.)
                if abs(center_y - img_h/2.0) < 20:
                    z_pid.SetPoint = center_y
                else:
                    z_pid.SetPoint = img_h / 2.0
                    
                z_pid.update(center_y)
                dy = z_pid.output
                z_dis += dy
                # Clamp z_dis to safe limits
                z_dis = 32.00 if z_dis > 32.00 else z_dis
                z_dis = 10.00 if z_dis < 10.00 else z_dis
                
                target = AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), -90, 90) # 逆运动学求解(inverse kinematics solution)
                if target: # 如果有解，则按照求出的解驱动舵机
                    servo_data = target[0]                  
                    # Only move if the change is significant
                    if abs(dx) > 2 or abs(dy) > 0.1:
                        board.pwm_servo_set_position(0.02, [[3, servo_data['servo3']],
                                                        [4, servo_data['servo4']],
                                                        [5, servo_data['servo5']],
                                                        [6, int(x_dis)]])
                        time.sleep(0.05)
    return img

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    # 实例化逆运动学库(instantiate the inverse kinematics library)
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    __isRunning = True
    __target_color = ('red')
    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
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
        time.sleep(0.01)  # 10ms delay per frame
    cv2.destroyAllWindows()
