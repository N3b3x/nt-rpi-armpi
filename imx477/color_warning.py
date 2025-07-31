#!/usr/bin/python3
# coding=utf8

import sys
import time
import cv2
import numpy as np
import os

# Custom module paths
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')

import common.yaml_handle as yaml_handle
from Camera import Camera
import lab_auto_calibration as lab_calib  # Modular calibration

# Check for Python 3
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
target_color = ('red', 'green', 'blue')
__isRunning = True
detect_color = 'None'
size = (640, 480)
draw_color = range_rgb["black"]
color_list = []

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
    print("Loaded LAB data:", lab_data)

def init():
    print("ColorWarning Init")
    load_config()

def start():
    global __isRunning
    __isRunning = True
    print("ColorWarning Start")

def stop():
    global __isRunning
    __isRunning = False
    print("ColorWarning Stop")

def run(img):
    global __isRunning, color_list, detect_color, draw_color

    if not __isRunning:
        return img

    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(cv2.GaussianBlur(frame_resize, (3, 3), 3), cv2.COLOR_BGR2LAB)

    # Show center LAB value
    h, w = frame_lab.shape[:2]
    center_lab = frame_lab[h//2, w//2]
    #print(f"Center LAB: {center_lab}")

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0

    # Show LAB masks live
    for i in lab_data:
        if i in target_color:
            mask = cv2.inRange(frame_lab, tuple(lab_data[i]['min']), tuple(lab_data[i]['max']))
            cv2.imshow(f"Mask - {i}", mask)  # 🟩 Show mask live!

            closed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            areaMaxContour, area_max = getAreaMaxContour(contours)

            if areaMaxContour is not None and area_max > max_area:
                max_area = area_max
                color_area_max = i
                areaMaxContour_max = areaMaxContour

    display_img = cv2.resize(img, size)

    if max_area > 2500:
        rect = cv2.minAreaRect(areaMaxContour_max)
        box = np.intp(cv2.boxPoints(rect))
        cv2.drawContours(display_img, [box], -1, range_rgb[color_area_max], 2)
        color_list.append({'red':1, 'green':2, 'blue':3}.get(color_area_max, 0))

        if len(color_list) == 3:
            color = int(round(np.mean(color_list)))
            color_list.clear()
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
        detect_color = 'None'
        draw_color = range_rgb["black"]

    cv2.putText(display_img, f"Detected Color: {detect_color}",
                 (10, display_img.shape[0] - 10),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    cv2.putText(display_img, "Press 'c' to recalibrate | Press 'q' to quit",
                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    return display_img

def getAreaMaxContour(contours):
    contour_area_max = 0
    areaMaxContour = None
    for c in contours:
        contour_area_temp = abs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max and contour_area_temp > 300:
            contour_area_max = contour_area_temp
            areaMaxContour = c
    return areaMaxContour, contour_area_max

if __name__ == '__main__':
    init()
    start()
    camera = Camera(resolution=(640, 480))

    while True:
        frame = camera.get_frame()
        if frame is not None:
            frame_rgb = frame[..., ::-1]
            Frame = run(frame_rgb)

            cv2.imshow('IMX477 Color Warning', Frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("Quitting...")
                break
            elif key == ord('c'):
                lab_calib.calibrate_lab_ranges(reference_image_path='reference_image.jpg',
                                               yaml_output_path=yaml_handle.lab_file_path_imx477)
                load_config()
                print("✅ Calibration done.")
        else:
            time.sleep(0.01)

    stop()
    cv2.destroyAllWindows()
