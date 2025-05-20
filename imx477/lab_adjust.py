#!/usr/bin/python3
# coding=utf8
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
import cv2
import math
import time
import numpy as np
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

__target_color = ('red',)
def setLABValue(lab_value):
    global lab_data
    global __target_color
    
    __target_color = (lab_value[0]['color'], )
    lab_data[__target_color[0]]['min'][0] = lab_value[0]['min'][0]
    lab_data[__target_color[0]]['min'][1] = lab_value[0]['min'][1]
    lab_data[__target_color[0]]['min'][2] = lab_value[0]['min'][2]
    lab_data[__target_color[0]]['max'][0] = lab_value[0]['max'][0]
    lab_data[__target_color[0]]['max'][1] = lab_value[0]['max'][1]
    lab_data[__target_color[0]]['max'][2] = lab_value[0]['max'][2]
    
    return (True, (), 'SetLABValue')

def getLABValue():
    _lab_value = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
    return (True, (_lab_value, ))

def saveLABValue(color):
    yaml_handle.save_yaml_data(lab_data, yaml_handle.lab_file_path_imx477)
    return (True, (), 'SaveLABValue')

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:
                area_max_contour = c

    return area_max_contour, contour_area_max

lab_data = None
def load_config():
    global lab_data
    print("yaml_handle loaded from:", yaml_handle.__file__)
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)

def reset():
    global __target_color
    __target_color = ()

def init():
    print("lab_adjust Init")
    load_config()
    reset()

__isRunning = False
def start():
    global __isRunning, __target_color
    __isRunning = True
    __target_color = ('red',)  # Set default target color
    print("lab_adjust Start")

def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("lab_adjust Stop")

def exit():
    global __isRunning
    __isRunning = False
    print("lab_adjust Exit")

def run(img):  
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img
    
    frame_gb = cv2.GaussianBlur(img_copy, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    
    # Get center pixel LAB value
    h, w = frame_lab.shape[:2]
    center_lab = frame_lab[h//2, w//2]
    
    # Display current LAB values
    blue = (255, 0, 0)
    cv2.putText(img_copy, f"Center LAB: {center_lab}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
    if __target_color and __target_color[0] in lab_data:
        cv2.putText(img_copy, f"Min LAB: {lab_data[__target_color[0]]['min']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
        cv2.putText(img_copy, f"Max LAB: {lab_data[__target_color[0]]['max']}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
    
    for i in lab_data:
        if i in __target_color:
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            frame_bgr = cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR)
            img = frame_bgr
    
    # Display controls help
    blue = (255, 0, 0)
    cv2.putText(img_copy, "L: +/- 1   A: +/- 1   B: +/- 1", (10, img_h - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
    cv2.putText(img_copy, "j/k: L -10/+10   n/m: A -10/+10   ,/.: B -10/+10", (10, img_h - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
    cv2.putText(img_copy, "S: Save, R: Reset, ESC: Quit", (10, img_h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue, 2)
    
    return img_copy

if __name__ == '__main__':      
    init()
    start()
    camera = Camera(resolution=(640, 480))
    while True:
        frame = camera.get_frame()
        if frame is not None:
            frame = frame[..., ::-1]  # This swaps R and B channels
            Frame = run(frame)           
            
            # Show both original and mask
            cv2.imshow('Original', frame)
            cv2.imshow('Mask', Frame)
            
            key = cv2.waitKey(1) & 0xFF  # Only keep lower 8 bits
            if key == 27:  # ESC
                break
            elif key == ord('s'):  # Save
                if __target_color and __target_color[0] in lab_data:
                    saveLABValue(__target_color[0])
                    print(f"Saved LAB values for {__target_color[0]}")
            elif key == ord('r'):  # Reset
                load_config()
                print("Reset to saved values")
            elif __target_color and __target_color[0] in lab_data:
                # Adjust L value
                if key == ord('l'):
                    lab_data[__target_color[0]]['min'][0] = max(0, lab_data[__target_color[0]]['min'][0] - 1)
                    lab_data[__target_color[0]]['max'][0] = min(255, lab_data[__target_color[0]]['max'][0] + 1)
                elif key == ord('j'):
                    lab_data[__target_color[0]]['min'][0] = max(0, lab_data[__target_color[0]]['min'][0] - 10)
                elif key == ord('k'):
                    lab_data[__target_color[0]]['max'][0] = min(255, lab_data[__target_color[0]]['max'][0] + 10)
                # Adjust A value
                elif key == ord('a'):
                    lab_data[__target_color[0]]['min'][1] = max(0, lab_data[__target_color[0]]['min'][1] - 1)
                    lab_data[__target_color[0]]['max'][1] = min(255, lab_data[__target_color[0]]['max'][1] + 1)
                elif key == ord('n'):
                    lab_data[__target_color[0]]['min'][1] = max(0, lab_data[__target_color[0]]['min'][1] - 10)
                elif key == ord('m'):
                    lab_data[__target_color[0]]['max'][1] = min(255, lab_data[__target_color[0]]['max'][1] + 10)
                # Adjust B value
                elif key == ord('b'):
                    lab_data[__target_color[0]]['min'][2] = max(0, lab_data[__target_color[0]]['min'][2] - 1)
                    lab_data[__target_color[0]]['max'][2] = min(255, lab_data[__target_color[0]]['max'][2] + 1)
                elif key == ord(','):
                    lab_data[__target_color[0]]['min'][2] = max(0, lab_data[__target_color[0]]['min'][2] - 10)
                elif key == ord('.'):
                    lab_data[__target_color[0]]['max'][2] = min(255, lab_data[__target_color[0]]['max'][2] + 10)
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows() 