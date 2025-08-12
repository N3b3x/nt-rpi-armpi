#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/pi/ArmPi_mini/')
import cv2
import time
import threading
import numpy as np
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:
    def __init__(self, resolution=(640, 480), enable_calibration=False):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False
        self.enable_calibration = enable_calibration
        
        # Only load calibration parameters if enabled
        if self.enable_calibration:
            try:
                # load parameter
                self.param_data = np.load(calibration_param_path + '.npz')        
                # obtain parameter
                self.dim = tuple(self.param_data['dim_array'])
                self.k = np.array(self.param_data['k_array'].tolist())
                self.d = np.array(self.param_data['d_array'].tolist())
                self.p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.k, self.d, self.dim ,None)
                self.Knew = self.p.copy()
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.k, self.d, np.eye(3), self.Knew, self.dim, cv2.CV_16SC2)
                print("Camera calibration loaded and enabled")
            except Exception as e:
                print(f"Warning: Could not load calibration data: {e}")
                self.enable_calibration = False
        else:
            print("Camera calibration disabled - using raw camera feed")
        
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self):
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.opened = True
        except Exception as e:
            print('Failed to open camera:', e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('Failed to close camera:', e)

    def set_calibration(self, enable):
        """Enable or disable calibration correction"""
        self.enable_calibration = enable
        if enable:
            try:
                self.param_data = np.load(calibration_param_path + '.npz')        
                self.dim = tuple(self.param_data['dim_array'])
                self.k = np.array(self.param_data['k_array'].tolist())
                self.d = np.array(self.param_data['d_array'].tolist())
                self.p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.k, self.d, self.dim ,None)
                self.Knew = self.p.copy()
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.k, self.d, np.eye(3), self.Knew, self.dim, cv2.CV_16SC2)
                print("Camera calibration enabled")
            except Exception as e:
                print(f"Error loading calibration: {e}")
                self.enable_calibration = False
        else:
            print("Camera calibration disabled")

    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():
                    ret, raw_img = self.cap.read()
                    if ret:
                        if self.enable_calibration and hasattr(self, 'map1') and hasattr(self, 'map2'):
                            # Apply calibration correction
                            correct_img = cv2.remap(raw_img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                            self.frame = cv2.resize(correct_img, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                        else:
                            # Use raw image without calibration
                            self.frame = cv2.resize(raw_img, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                    else:
                        self.frame = None
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('Error getting camera frame:', e)
                time.sleep(0.01)

if __name__ == '__main__':
    my_camera = Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            cv2.imshow('img', img)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
