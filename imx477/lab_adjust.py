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
from Camera import Camera
import lab_auto_calibration  # ⬅️ Modular calibration

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
    
    # Show key instructions
    instructions = [
        "KEYS: [1]-Red, [2]-Green, [3]-Blue, [p]-PhotoRef, [c]-ColorCalib, [d]-DistCalib, [q]-Quit",
    ]
    for idx, text in enumerate(instructions):
        cv2.putText(img_copy, text, (10, h - 20 - 30 * idx),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # Show center LAB and target color
    cv2.putText(img_copy, f"Center LAB: {center_lab}", (10, 30), 
                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    if __target_color and __target_color[0] in lab_data:
        cv2.putText(img_copy, f"Target: {__target_color[0]}", (10, 60), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    return mask_display, img_copy

if __name__ == '__main__':
    init()
    start()
    camera = Camera(resolution=(640, 480))

    calib_image_counter = 1

    while True:
        frame = camera.get_frame()
        if frame is not None:
            frame = frame[..., ::-1]
            mask, original = run(frame)
            cv2.imshow('Original', original)
            cv2.imshow('Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
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
                    frame_ref = camera.get_frame()
                    if frame_ref is not None:
                        frame_rgb = frame_ref[..., ::-1]
                        cv2.imshow("Reference Capture - Press 'c' to capture", frame_rgb)
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
                    frame_cb = camera.get_frame()
                    if frame_cb is not None:
                        frame_bgr = frame_cb  # Keep as BGR for OpenCV functions
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
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
