#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import sys
import numpy as np
import threading
import mediapipe as mp
from common import yaml_handle
from common.ros_robot_controller_sdk import Board

# Chapter 6 AI Vision Project Course/2.OpenCV Vision Application/Lesson 7 Face Detection

debug = False

iHWSONAR = None
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
# import facial recognition module
Face = mp.solutions.face_detection
# Customize face recognition method, and the minimum face detection confidence is 0.5
faceDetection = Face.FaceDetection(min_detection_confidence=0.8)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

# the closing angle of the gripper while grasping an object
servo1 = 1500

# initial position
def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90,1500)
    

start_greet = False
action_finish = True
# reset variables
def reset():
    global d_pulse
    global start_greet
    global x_pulse    
    global action_finish

 
    start_greet = False
    action_finish = True
    x_pulse = 500 
    init_move()  

__isRunning = False

# call the initialization of the app
def init():
    print("ColorDetect Init")
    load_config()
    initMove()


# the app starts the game calling
def start():
    global __isRunning
    __isRunning = True
    print("ColorDetect Start")

def move():
    global start_greet
    global action_finish
    
    while True:
        if __isRunning:
            if start_greet:
                start_greet = False
                action_finish = False
                board.set_buzzer(1900, 0.1, 0.3, 2)# set the buzzer to sound for 0.1s
                time.sleep(0.9)
                
                action_finish = True
                
            else:
                time.sleep(0.2)
        else:
            time.sleep(0.2)
    

# run sub-thread
threading.Thread(target=move, args=(), daemon=True).start()


size = (320, 240)
def run(img):
    global __isRunning, area
    global center_x, center_y
    global center_x, center_y, area
    global start_greet
    global action_finish
    if not __isRunning:   # Detect if the game is started, if not, return the original image
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
     
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # convert BGR image to RGB image
    results = faceDetection.process(imgRGB) # transmit the image of each frame to facial recognition module

    if results.detections:  # If no face is detected, return None

        for index, detection in enumerate(results.detections):  # Return the face index (which face) and the coordinate information of the keypoints
            scores = list(detection.score)
            if scores and scores[0] > 0.7:
                bboxC = detection.location_data.relative_bounding_box  # Set a bounding box to receive xywh and keypoint information for all received boxes
                
                # Convert the coordinates' width and height of the bounding box from proportional coordinates to pixel coordinates
                bbox = (
                    int(bboxC.xmin * img_w),
                    int(bboxC.ymin * img_h),
                    int(bboxC.width * img_w),
                    int(bboxC.height * img_h)
                )
                
                cv2.rectangle(img, bbox, (0, 255, 0), 2)  # draw a rectangle on each frame of the image
                
                # Get information about the recognition box, where xy is the coordinates of the upper left corner
                x, y, w, h = bbox
                center_x = int(x + (w / 2))
                center_y = int(y + (h / 2))
                area = int(w * h)
                if action_finish:
                    start_greet = True


    else:
        center_x, center_y, area = -1, -1, 0
            
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
    my_camera.camera_close()
    cv2.destroyAllWindows()