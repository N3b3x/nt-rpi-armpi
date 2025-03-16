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

#  6.AI视觉学习课程/第5课 智能码垛(6.AI Vision Games Lesson/Lesson 5 Intelligent Stacking)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# 读取夹取坐标(read gripping coordinate)
Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

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

# 设置检测的目标颜色(set detected target color)
__target_color = ('red', 'green', 'blue')
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
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(Only the contour with the area larger than 300, which is greater than 300, is considered valid to filter out disturbance.)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the largest contour)

# 夹持器夹取时闭合的角度(the closing angle of the gripper while grasping an object)
servo1 = 1500

# 初始位置(initial position)
def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 8, 10), -90,-90, 90, 1500)

   
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
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False

# 变量重置(reset variables)
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

# 初始化(initialization)
def init():
    global number
    number = 0
    print("ColorPalletizing Init")
    load_config()
    initMove()

# APP开启玩法(app starts the game)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorPalletizing Start")

# APP关闭玩法(app stops the game)
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorPalletizing Stop")

# APP退出玩法(app exits the game)
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorPalletizing Exit")


size = (320, 240)
# 机械臂移动函数(function for the robotic arm's movement)
def move():
    global _stop
    global get_roi
    global number
    global __isRunning
    global detect_color
    global start_pick_up
    
    x = Coordinates_data['X']
    y = Coordinates_data['Y']
    z = Coordinates_data['Z']
    
    coordinate = {
        'capture': (x, y, z),       # 夹取坐标(gripping coordinate)
        'place': (12, 0, 0.5), # 放置坐标(placing coordinate)
        }
    
    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # 如果检测到方块没有移动一段时间后，开始夹取(if the block is detected no moving for a period, the robotic arm starts to grip)
                set_rgb(detect_color) # 设置扩展板上的RGB灯亮对应的颜色(set the RGB light on the expansion board to light corresponding color)
                board.set_buzzer(1900, 0.1, 0.9, 1)# 设置蜂鸣器响0.1秒(set the buzzer to sound for 0.1s)
                
                board.pwm_servo_set_position(0.5, [[1, 2000]])# 张开爪子(open gripper)
                time.sleep(0.6)
                if not __isRunning: # 检测是否停止玩法(detect if the game is stopped)
                    continue
                AK.setPitchRangeMoving((coordinate['capture']), -90,-90, 90, 1000) # 机械臂运行到夹取位置(robotic arm runs to the gripping position)
                time.sleep(1) # 延时1秒(delay for 1s)
                if not __isRunning:
                    continue
                board.pwm_servo_set_position(0.5, [[1, 1450]])# 闭合爪子(close the gripper)
                time.sleep(0.6)
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) # 抬起机械臂(uplift the robotic arm)
                time.sleep(1.5)
                if not __isRunning:
                    continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])# 先把机械臂转过去(rotate the robotic arm to the position first)
                time.sleep(1.5)
                if not __isRunning:
                    continue
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1],12), -90,-90, 90, 800) # 机械臂运行到夹取位置上方(run the robotic arm to the top of the gripping position)
                time.sleep(0.8)
                if not __isRunning:
                    continue
                
                AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1],(coordinate['place'][2]+number*3)), -90,-90, 90, 800) # 机械臂运行到放置位置,根据码垛的数量计算放置坐标的高度(robotic arm runs to the placing position, the height of the placing position is calculated based on numbers of the stacked blocks)
                time.sleep(0.8)
                if not __isRunning: 
                    continue
                board.pwm_servo_set_position(0.5, [[1, 1800]]) # 张开爪子,放下木块(open the gripper to put down the block)
                time.sleep(0.6)
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((6, 0, 18), 0,-90, 90, 1500) # 抬起机械臂(uplift the robotic arm)
                time.sleep(1.5)
                if not __isRunning:
                    continue
                board.pwm_servo_set_position(0.5, [[6, 1500]])  # 机械臂转回中间(robotic arm returns to the middle)
                time.sleep(1.5)
                
                if not __isRunning:
                    continue
                AK.setPitchRangeMoving((0, 8, 10), -90,-90, 90, 800) # 机械臂复位(reset the robotic arm)
                time.sleep(0.8)
                 
                number += 1 # 码垛数量累加(accumulate the number of stacking)
                if number == 3: # 如果等于3，进行计数重置(if it equals to 3, reset the counting)
                    number = 0
                    board.set_buzzer(1900, 0.1, 0.9, 1)# 设置蜂鸣器响0.1秒(set the buzzer to sound for 0.1s)
                    set_rgb('white')
                    time.sleep(0.5)
                    
                if not __isRunning:
                    continue
                # 对应变量进行重置(reset corresponding variables)
                detect_color = 'None'
                get_roi = False
                start_pick_up = False
                set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                initMove()  # 回到初始位置(return to initial position)
            time.sleep(0.01)

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()      


roi = ()
center_list = []
draw_color = range_rgb["black"]
# 图像处理(image processing)
def run(img):
    global roi
    global count
    global get_roi
    global center_list
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
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

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
                                                  lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation on the original image and the mask)
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算(opening operation)
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算(closing operation)
                    closed[0:80, :] = 0
                    closed[:, 0:120] = 0
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
                    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find the largest contour)
                    if areaMaxContour is not None:
                        if area_max > max_area:  # 找最大面积(find the maximum area)
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 500:  # 有找到最大面积(the maximum area has been found)
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum circumscribed circle)
                center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
                center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))
                cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
                
                if not start_pick_up:
                    if color_area_max == 'red':  # 红色最大(red is the maximum)
                        color = 1
                    elif color_area_max == 'green':  # 绿色最大(green is the maximum)
                        color = 2
                    elif color_area_max == 'blue':  # 蓝色最大(blue is the maximum)
                        color = 3
                        color = 3
                    else:
                        color = 0
                    color_list.append(color)
                    if len(color_list) == 3:  # 多次判断(multiple judgements)
                        # 取平均值(get mean)
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
    # 实例化逆运动学库(instantiate the inverse kinematics library)
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
