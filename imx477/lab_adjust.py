#!/usr/bin/python3
# coding=utf8

import os
import shutil
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
import cv2
import time
import numpy as np
import common.yaml_handle as yaml_handle
from picamera2 import Picamera2
import lab_auto_calibration  # ⬅️ Modular calibration
from Camera import Camera  # Fixed import path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'color_palletizing'))
from color_palletizing.arm_controller import ArmController

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
}

__target_color = ('red',)
lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path_imx477)
    print("Loaded LAB data:", lab_data)

def reset():
    global __target_color
    __target_color = ()

def init():
    print("lab_adjust Init")
    load_config()

__isRunning = False
def start():
    global __isRunning, __target_color
    __isRunning = True
    __target_color = ('red',)
    print("lab_adjust Start")

def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("lab_adjust Stop")

def run(img):  
    img_copy = img.copy()
    frame_lab = cv2.cvtColor(cv2.GaussianBlur(img_copy, (3, 3), 3), cv2.COLOR_BGR2LAB)
    h, w = frame_lab.shape[:2]
    center_lab = frame_lab[h//2, w//2]
    combined_mask = np.zeros(frame_lab.shape[:2], dtype=np.uint8)
    
    for i in ['blue', 'green', 'red']:
        if i in lab_data:
            mask = cv2.inRange(frame_lab, tuple(lab_data[i]['min']), tuple(lab_data[i]['max']))
            if i == 'red':
                combined_mask[mask > 0] = 1
            elif i == 'green':
                combined_mask[mask > 0] = 2
            elif i == 'blue':
                combined_mask[mask > 0] = 3
    
    mask_display = np.zeros((h, w, 3), dtype=np.uint8)
    mask_display[combined_mask == 1] = [0, 0, 255]
    mask_display[combined_mask == 2] = [0, 255, 0]
    mask_display[combined_mask == 3] = [255, 0, 0]
    
    # Show key instructions (brighter text, split into two lines for better readability)
    instructions = [
        "KEYS: [1]-Red, [2]-Green, [3]-Blue, [p]-PhotoRef, [c]-ColorCalib,",
        "[d]-DistCalib, [x]-CoordCalib, [m]-ManualMode, [q]-Quit"
    ]
    for idx, text in enumerate(instructions):
        cv2.putText(img_copy, text, (10, 70 + 30 * idx),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

    # Show center LAB and target color
    cv2.putText(img_copy, f"Center LAB: {center_lab}", (10, 30), 
                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    if __target_color and __target_color[0] in lab_data:
        cv2.putText(img_copy, f"Target: {__target_color[0]}", (10, 45), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return mask_display, img_copy

def manual_camera_controls(picam2):
    exposure = 30000  # or higher
    gain = 4.0
    brightness = 0.2
    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame_bgr.shape[:2]

            # Display settings
            instructions = [
                "Manual Camera Controls - Keys:",
                "[e] Exposure-  [r] Exposure+",
                "[g] Gain-  [h] Gain+",
                "[b] Brightness-  [n] Brightness+",
                "[m] Apply  |  [q] Quit Controls"
            ]
            for idx, text in enumerate(instructions):
                cv2.putText(frame_bgr, text, (10, 30 + 25 * idx),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Always show the current values just below the instructions
            cv2.putText(
                frame_bgr,
                f"Exposure: {exposure} | Gain: {gain:.2f} | Brightness: {brightness:.2f}",
                (10, 30 + 25 * len(instructions)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
            )

            cv2.imshow("Camera Controls", frame_bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Exiting camera controls...")
                cv2.destroyWindow("Camera Controls")
                break
            elif key == ord('e'):
                exposure = max(1000, exposure - 1000)
            elif key == ord('r'):
                exposure += 1000
            elif key == ord('g'):
                gain = max(1.0, gain - 0.1)
            elif key == ord('h'):
                gain += 0.1
            elif key == ord('b'):
                brightness = max(-1.0, brightness - 0.1)
            elif key == ord('n'):
                brightness = min(1.0, brightness + 0.1)
            elif key == ord('m'):
                lab_auto_calibration.set_controls(
                    picam2,
                    awb_enable=False,
                    awb_mode=0,
                    colour_gains=(1.2, 2.2),
                    ae_enable=False,
                    exposure_time=exposure,
                    analogue_gain=gain,
                    sharpness=8.0,
                    contrast=1.0,
                    saturation=1.0,
                    brightness=brightness
                )
                print("? Camera settings applied!")

def load_calibration_data(path='calibration_data.npz'):
    data = np.load(path)
    K = data['K']
    D = data['D']
    return K, D

if __name__ == '__main__':
    init()
    start()
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    arm_controller = ArmController()  # Create arm controller instance

    calib_image_counter = 1
    undistort_enabled = False
    K, D = None, None
    if os.path.exists('calibration_data.npz'):
        K, D = load_calibration_data('calibration_data.npz')

    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if undistort_enabled and K is not None and D is not None:
                frame_bgr = lab_auto_calibration.undistort_frame(frame_bgr, K, D)
            mask, original = run(frame_bgr)
            cv2.imshow('Original', original)
            cv2.imshow('Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('u'):
                undistort_enabled = not undistort_enabled
                print(f"Undistortion {'enabled' if undistort_enabled else 'disabled'}")
            elif key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('1'):
                __target_color = ('red',)
                print("Selected Red")
            elif key == ord('2'):
                __target_color = ('green',)
                print("Selected Green")
            elif key == ord('3'):
                __target_color = ('blue',)
                print("Selected Blue")
            elif key == ord('p'):
                print("\n📸 Please hold up the ColorChecker chart clearly in front of the camera.")
                print("Press 'c' to capture the reference image or 'q' to cancel.")
                while True:
                    frame_ref = picam2.capture_array()
                    if frame_ref is not None:
                        frame_bgr = cv2.cvtColor(frame_ref, cv2.COLOR_RGB2BGR)
                        cv2.imshow("Reference Capture - Press 'c' to capture", frame_bgr)
                        capture_key = cv2.waitKey(1) & 0xFF
                        if capture_key == ord('c'):
                            lab_auto_calibration.capture_reference_image(frame_ref, save_path='reference_image.jpg')
                            print("✅ Reference image captured and saved.")
                            break
                        elif capture_key == ord('q'):
                            print("Reference capture cancelled.")
                            break
                    else:
                        time.sleep(0.1)
                cv2.destroyWindow("Reference Capture - Press 'c' to capture")
            elif key == ord('c'):
                lab_auto_calibration.calibrate_lab_ranges(
                    reference_image_path='reference_image.jpg',
                    yaml_output_path=yaml_handle.lab_file_path_imx477
                )
                load_config()
                print("✅ Calibration complete. LAB ranges updated!")
            elif key == ord('d'):
                print("\n🔍 Please hold up the checkerboard for distortion calibration.")
                print("Move the checkerboard around slightly and take at least 10 images from different angles.")
                print("Press 'c' to capture calibration images, 'q' to finish capture.")

                image_dir = 'calib_images'
                if os.path.exists(image_dir):
                    shutil.rmtree(image_dir)
                os.makedirs(image_dir, exist_ok=True)

                img_counter = 0

                while True:
                    frame_cb = picam2.capture_array()
                    if frame_cb is not None:
                        frame_bgr = cv2.cvtColor(frame_cb, cv2.COLOR_RGB2BGR)
                        instructions = "Move checkerboard. Press 'c' to capture, 'q' to finish."
                        cv2.putText(frame_bgr, instructions, (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.imshow("Checkerboard Capture", frame_bgr)

                        key_cb = cv2.waitKey(1) & 0xFF
                        if key_cb == ord('c'):
                            img_counter += 1
                            img_name = os.path.join(image_dir, f"calib_{img_counter:03d}.jpg")
                            cv2.imwrite(img_name, frame_cb)
                            print(f"✅ Calibration image saved as {img_name}")
                        elif key_cb == ord('q'):
                            print("✅ Calibration image capture finished. Starting calibration...")
                            break
                    else:
                        time.sleep(0.1)

                cv2.destroyWindow("Checkerboard Capture")

                lab_auto_calibration.calibrate_camera(
                    image_dir=image_dir,
                    checkerboard=(6, 9),
                    save_path='calibration_data.npz'
                )
                print("✅ Distortion calibration complete.")
                # Reload calibration data after calibration
                if os.path.exists('calibration_data.npz'):
                    K, D = load_calibration_data('calibration_data.npz')
            elif key == ord('x'):  # New option for coordinate calibration
                print("\n🔍 Starting coordinate calibration...")
                lab_auto_calibration.calibrate_coordinates(
                    get_frame=lambda: cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR),
                    checkerboard=(6, 9),
                    square_size=20.0,  # Adjust this to match your checkerboard square size
                    save_path='coordinate_calibration.npz',
                    arm_controller=arm_controller
                )
            elif key == ord('m'):
                manual_camera_controls(picam2)
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
