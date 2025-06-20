#!/usr/bin/python3
# coding=utf8

import os
import shutil
import sys
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/yaml')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/CameraCalibration')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')  # Add kinematics_sdk path
sys.path.append('/home/pi/ArmPi_mini/')
import cv2
import time
import numpy as np
import common.yaml_handle as yaml_handle
from picamera2 import Picamera2
import lab_auto_calibration  # ⬅️ Modular calibration
from Camera import Camera  # Fixed import path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'color_palletizing'))
from color_palletizing.arm_controller import ArmController, SERVO_GRIPPER, SERVO_ELBOW, SERVO_SHOULDER, SERVO_LIFT, SERVO_BASE
from color_palletizing.camera_processor import CameraProcessor
import math
import webbrowser
from help_manager import HelpManager

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
}

__target_color = ('red',)
lab_data = None

# Global variables for 3D calibration
calibration_points = []
clicked_pixel = None
calibration_image = None

# Pre-defined world coordinates for 3D calibration
# The user will be prompted to move the arm to touch these physical points.
CALIBRATION_GRID_POINTS = [
    (15, 5, 0.5),
    (20, 5, 0.5),
    (15, -5, 0.5),
    (20, -5, 0.5),
    (18, 0, 5),   # A higher point
    (15, 0, 10),  # An even higher point
]

CALIBRATION_INSTRUCTIONS = """
# ArmPi Mini Calibration Guide

## Objective
To accurately calibrate the camera so the system can translate a pixel seen by the camera into a real-world (X, Y, Z) coordinate for the robotic arm.

## Required Items
*   A printed checkerboard pattern.
*   A calibration grid: a piece of paper with the following points marked in centimeters, matching the `CALIBRATION_GRID_POINTS` in the code:
    *   `(15, 5)`, `(20, 5)`, `(15, -5)`, `(20, -5)` on the table surface.
*   You'll also need a way to touch points at a height of `5cm` and `10cm` (e.g., placing a block of a known height at a marked spot).
*   (Optional but recommended) A color reference chart, like a ColorChecker, for color calibration.

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Camera Distortion Calibration (Key: `d`)
This step corrects the "fisheye" effect of the lens.

1.  Press the `d` key.
2.  The script will prompt you to show a checkerboard to the camera.
3.  Hold the checkerboard flat and in full view of the camera.
4.  Move the checkerboard to different positions and angles (top, bottom, left, right, tilted). At each position, press the `c` key to capture an image.
5.  Capture at least 10-15 good images.
6.  Once you are done, press the `q` key to finish capturing.
7.  The script will process the images and save the results to a file named `calibration_data.npz`. This file contains the essential camera matrix (`K`) and distortion coefficients (`D`).

---

## Step 3: Guided 3D Pose Calibration (Key: `y`)
This is the most important step. It tells the system exactly where the camera is located and how it's oriented in relation to the arm's base.

1.  Press the `y` key.
2.  The script will display **"STEP 1: JOG ARM"** and show the first target coordinate from the predefined list (e.g., `(15, 5, 0.5)`).
3.  Using your marked paper grid as a reference, use the jog keys to manually move the arm until the very tip of the gripper is physically touching that exact point in the real world.
    *   **Jog Keys:** `w/s` (Lift), `a/d` (Shoulder), `q/e` (Elbow), `z/c` (Base).
4.  Once the arm is perfectly positioned, press the **spacebar**.
5.  The display will change to **"STEP 2: CLICK PIXEL"**. Now, use your mouse to click on the live video feed at the exact pixel where the gripper's tip is. A green circle will appear to confirm your click.
6.  If you are satisfied with the click location, press `y` to confirm and save this calibration point.
7.  If you made a mistake (the arm wasn't positioned right or you clicked the wrong spot), press `n` to restart the process for this *same* point.
8.  The script will automatically move to the next target coordinate in the list. Repeat the `jog -> spacebar -> click -> confirm (y)` process for all points.
9.  After completing all points, the script will calculate the camera's 3D position and save the result to `3d_camera_pose.npz`.

---

## Step 4: Test the Calibration (Key: `t`)
This step verifies the accuracy of your calibration.

1.  Press the `t` key.
2.  The script will load the `3d_camera_pose.npz` file and run a test calculation.
3.  It will print a "Test conversion error" value in the terminal. This number represents the average distance (in cm) between the real-world points you provided and the points the system calculated based on your pixel clicks.
4.  **A good calibration should have an error well below 1.0 cm.** If the error is high, you should re-run the Guided 3D Pose Calibration (Step 3), being more precise with your jogging and clicking.

---
After completing these steps, your system will be fully calibrated and ready to translate camera detections into arm movements.
"""

def show_instructions_in_browser():
    """Saves instructions to a markdown file and opens it in a web browser."""
    instructions_file = "calibration_instructions.md"
    with open(instructions_file, "w") as f:
        f.write(CALIBRATION_INSTRUCTIONS)
    
    # Get the full path to the file
    file_path = os.path.abspath(instructions_file)
    
    # Open in the default web browser
    webbrowser.open_new_tab(f'file://{file_path}')
    print(f"✅ Opened calibration instructions in your web browser.")

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
        "[d]-DistCalib, [j]-Jog, [y]-3DGripperCalib, [t]-Test3D, [m]-CamManualMode, [q]-Quit"
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

def mouse_callback(event, x, y, flags, param):
    """Mouse callback for pixel coordinate selection during 3D calibration."""
    global clicked_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_pixel = (x, y)
        print(f"✅ Clicked pixel coordinates: ({x}, {y})")

if __name__ == '__main__':
    init()
    start()
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    arm_controller = ArmController()  # Create arm controller instance
    camera_processor = CameraProcessor(picam2=picam2)  # Create camera processor instance
    help_manager = HelpManager() # Create help manager instance

    calib_image_counter = 1
    undistort_enabled = False
    K, D = None, None
    if os.path.exists('calibration_data.npz'):
        K, D = camera_processor.load_calibration_data('calibration_data.npz')

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
                image_dir = 'calib_images'
                recapture = True  # Default to recapturing

                # Check if images already exist and ask the user what to do
                if os.path.exists(image_dir) and os.listdir(image_dir):
                    print("\n📸 Found existing calibration images.")
                    print("❓ Press 'y' to REUSE them, or 'n' to RECAPTURE new ones.")
                    
                    # Display prompt on the main window and wait for user input
                    prompt_loop = True
                    while prompt_loop:
                        frame_prompt = picam2.capture_array()
                        if frame_prompt is not None:
                            frame_bgr = cv2.cvtColor(frame_prompt, cv2.COLOR_RGB2BGR)
                            if undistort_enabled and K is not None and D is not None:
                                frame_bgr = lab_auto_calibration.undistort_frame(frame_bgr, K, D)
                            
                            cv2.putText(frame_bgr, "Reuse images? (y/n)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                            cv2.imshow('Original', frame_bgr)

                        key_choice = cv2.waitKey(1) & 0xFF
                        if key_choice == ord('y'):
                            recapture = False
                            prompt_loop = False
                            print("👍 Reusing existing images.")
                        elif key_choice == ord('n'):
                            recapture = True
                            prompt_loop = False
                            print("🗑️  Will capture new images.")
                
                if recapture:
                    print("\n🔍 Starting Distortion Calibration in High Resolution...")
                    print("Please hold up the checkerboard for distortion calibration.")
                    print("Move the checkerboard around and take at least 10-15 images from different angles.")
                    print("Press 'c' to capture calibration images, 'q' to finish capture.")

                    # Switch to higher resolution for better accuracy
                    print("Switching to 1920x1080 resolution...")
                    picam2.stop()
                    time.sleep(0.5)
                    config_calib = picam2.create_preview_configuration(
                        main={"size": (1920, 1080)},
                        controls={"FrameDurationLimits": (33333, 33333)} # ~30fps
                    )
                    picam2.configure(config_calib)
                    picam2.start()
                    time.sleep(2) # Allow camera to settle

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
                                cv2.imwrite(img_name, frame_bgr)
                                print(f"✅ Calibration image saved as {img_name}")
                            elif key_cb == ord('q'):
                                print("✅ Calibration image capture finished. Starting calibration...")
                                break
                        else:
                            time.sleep(0.1)

                    cv2.destroyWindow("Checkerboard Capture")

                    # Switch back to original resolution before heavy processing
                    print("Switching back to original preview resolution...")
                    picam2.stop()
                    time.sleep(0.5)
                    picam2.configure(config) # Revert to the original config
                    picam2.start()
                    time.sleep(1)

                lab_auto_calibration.calibrate_camera(
                    image_dir=image_dir,
                    checkerboard=(6, 9),
                    save_path='calibration_data.npz'
                )
                print("✅ Distortion calibration complete.")
                # Reload calibration data after calibration
                if os.path.exists('calibration_data.npz'):
                    K, D = camera_processor.load_calibration_data('calibration_data.npz')
            elif key == ord('j'):  # New option for jog mode
                print("\n🔍 Starting with jogging controls in high resolution...")
                print("Use keyboard controls to position the arm:")
                print("  [f/g] - Gripper open/close")
                print("  [q/e] - Elbow up/down") 
                print("  [a/d] - Shoulder forward/back")
                print("  [w/s] - Lift up/down")
                print("  [z/c] - Base rotate left/right")
                print("  [r] - Reset to center position")
                print("  [j] - Exit Jog mode")
                
                # Switch to higher resolution for better accuracy
                picam2.stop()
                time.sleep(1)
                config = picam2.create_preview_configuration(
                    main={"size": (1920, 1080)},
                    controls={"FrameDurationLimits": (33333, 33333)}
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(2)

                # Initialize arm to ready position (like Hiwonder sample)
                arm_controller.init_move()

                # After init_move(), the jog commands will start from the ready position's
                # joint angles, but the angle variables are not updated by init_move.
                # For now, jogging will be relative to a presumed zero-angle state.
                # A more robust solution would involve updating the angle state after IK movements.
                # arm_controller.current_lift_angle = 0
                # arm_controller.current_shoulder_angle = 0
                # arm_controller.current_elbow_angle = 0
                # arm_controller.current_base_angle = 0
                # arm_controller.current_gripper_pos = 1500

                jog_mode = True
                while jog_mode:
                    frame_jog = picam2.capture_array()
                    if frame_jog is not None:
                        frame_bgr = cv2.cvtColor(frame_jog, cv2.COLOR_RGB2BGR)
                        
                        # Define text color
                        blue_color = (255, 128, 0) # Medium Blue (BGR)

                        # Draw jogging instructions on frame, now at the bottom-left
                        instructions_with_angles = [
                            f"[f/g] Gripper: {arm_controller.current_gripper_pos}",
                            f"[q/e] Elbow: {arm_controller.current_elbow_angle:.1f} deg",
                            f"[a/d] Shoulder: {arm_controller.current_shoulder_angle:.1f} deg",
                            f"[w/s] Lift: {arm_controller.current_lift_angle:.1f} deg",
                            f"[z/c] Base: {arm_controller.current_base_angle:.1f} deg",
                            "",
                            "[r] Reset to center",
                            "[j] Exit Jog Mode"
                        ]
                        
                        y0, dy = frame_bgr.shape[0] - (len(instructions_with_angles) * 25 + 20), 25
                        for i, text in enumerate(instructions_with_angles):
                            y = y0 + i * dy
                            cv2.putText(frame_bgr, text, (10, y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, blue_color, 2)
                        
                        cv2.imshow("Jog Mode", frame_bgr)
                        
                        jog_key = cv2.waitKey(1) & 0xFF
                        if jog_key == ord('j'):
                            print("Exiting Jog Mode...")
                            jog_mode = False
                        elif jog_key == ord('f'):
                            arm_controller.move_gripper(50) # Increased value for noticeable movement
                        elif jog_key == ord('g'):
                            arm_controller.move_gripper(-50) # Increased value for noticeable movement
                        elif jog_key == ord('q'):
                            arm_controller.move_elbow(2)
                        elif jog_key == ord('e'):
                            arm_controller.move_elbow(-2)
                        elif jog_key == ord('a'):
                            arm_controller.move_shoulder(-2)
                        elif jog_key == ord('d'):
                            arm_controller.move_shoulder(2)
                        elif jog_key == ord('w'):
                            arm_controller.move_lift(2)
                        elif jog_key == ord('s'):
                            arm_controller.move_lift(-2)
                        elif jog_key == ord('z'):
                            arm_controller.move_base(-2)
                        elif jog_key == ord('c'):
                            arm_controller.move_base(2)
                        elif jog_key == ord('r'):
                            # Reset to ready position
                            arm_controller.init_move() # init_move now correctly sets angles
                    else:
                        time.sleep(0.01)
                
                cv2.destroyWindow("Jog Mode")

                # Switch back to original resolution
                print("Switching back to original preview resolution...")
                picam2.stop()
                time.sleep(1)
                config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
                picam2.configure(config)
                picam2.start()
                time.sleep(1)
                
                print("Jogging mode finished.")
            elif key == ord('y'):  # 3D Gripper Real World Calibration
                print("\n🎯 Starting Guided 3D Gripper Calibration...")
                print("This mode uses higher resolution for accuracy.")
                print(f"We will guide you through touching {len(CALIBRATION_GRID_POINTS)} known physical points.")
                print("Press [q] at any time to cancel.")

                # Switch to higher resolution for better accuracy
                picam2.stop()
                time.sleep(1)
                config = picam2.create_preview_configuration(
                    main={"size": (1920, 1080)},
                    controls={"FrameDurationLimits": (33333, 33333)}
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(2)

                # Initialize calibration data
                calibration_points = []
                cv2.namedWindow("3D Calibration")
                cv2.setMouseCallback("3D Calibration", camera_processor.mouse_callback)

                # Initialize arm to ready position
                arm_controller.init_move()
                time.sleep(2)

                calib_mode = True
                point_index = 0
                while calib_mode and point_index < len(CALIBRATION_GRID_POINTS):
                    target_world_pos = CALIBRATION_GRID_POINTS[point_index]
                    camera_processor.clicked_pixel = None  # Reset for each point

                    # Stage 1: Jog arm to position
                    prompt_user_to_move = True
                    while prompt_user_to_move:
                        frame_calib = picam2.capture_array()
                        if frame_calib is None: continue
                        frame_bgr = cv2.cvtColor(frame_calib, cv2.COLOR_RGB2BGR)

                        instructions = [
                            f"POINT {point_index + 1}/{len(CALIBRATION_GRID_POINTS)} - STEP 1: JOG ARM",
                            f"TARGET (X,Y,Z): {target_world_pos} cm",
                            "", "JOG: [q/e]Elbow [a/d]Shoulder [w/s]Lift [z/c]Base",
                            "Once gripper touches the point, press [SPACE]."
                        ]
                        for i, text in enumerate(instructions):
                            cv2.putText(frame_bgr, text, (10, 30 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        cv2.putText(frame_bgr, "Press [i] for detailed instructions", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.imshow("3D Calibration", frame_bgr)
                        calib_key = cv2.waitKey(1) & 0xFF

                        if calib_key == ord('i'):
                            help_manager.show_calibration_help()
                        elif calib_key == ord('q'): calib_mode = False; prompt_user_to_move = False
                        elif calib_key == ord(' '): prompt_user_to_move = False
                        elif calib_key in [ord(c) for c in "qweasdzc"]: # Jog keys
                            if calib_key == ord('q'): arm_controller.move_elbow(2)
                            elif calib_key == ord('e'): arm_controller.move_elbow(-2)
                            elif calib_key == ord('a'): arm_controller.move_shoulder(-2)
                            elif calib_key == ord('d'): arm_controller.move_shoulder(2)
                            elif calib_key == ord('w'): arm_controller.move_lift(2)
                            elif calib_key == ord('s'): arm_controller.move_lift(-2)
                            elif calib_key == ord('z'): arm_controller.move_base(-2)
                            elif calib_key == ord('c'): arm_controller.move_base(2)

                    if not calib_mode: break

                    # Stage 2: Click the corresponding pixel
                    prompt_user_to_click = True
                    while prompt_user_to_click:
                        frame_calib = picam2.capture_array()
                        if frame_calib is None: continue
                        frame_bgr = cv2.cvtColor(frame_calib, cv2.COLOR_RGB2BGR)

                        instructions = [
                            f"POINT {point_index + 1}/{len(CALIBRATION_GRID_POINTS)} - STEP 2: CLICK PIXEL",
                            "Click the exact pixel where the gripper tip is.",
                            "A green circle will appear.", "",
                            "Press [y] to confirm point.",
                            "Press [n] to re-jog this point."
                        ]
                        for i, text in enumerate(instructions):
                             cv2.putText(frame_bgr, text, (10, 30 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                        if camera_processor.clicked_pixel:
                            cv2.circle(frame_bgr, camera_processor.clicked_pixel, 10, (0, 255, 0), 2)
                        
                        cv2.putText(frame_bgr, "Press [i] for detailed instructions", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.imshow("3D Calibration", frame_bgr)
                        click_key = cv2.waitKey(1) & 0xFF

                        if click_key == ord('i'):
                            help_manager.show_calibration_help()
                        elif click_key == ord('q'): calib_mode = False; prompt_user_to_click = False
                        elif click_key == ord('n'): prompt_user_to_click = False
                        elif click_key == ord('y'):
                            if camera_processor.clicked_pixel:
                                clicked_pixel = camera_processor.clicked_pixel
                                calibration_points.append((target_world_pos, clicked_pixel))
                                print(f"✅ Point {point_index + 1} recorded: World{target_world_pos} -> Pixel{clicked_pixel}")
                                point_index += 1
                                prompt_user_to_click = False
                            else:
                                print("⚠️ Please click a pixel before confirming.")
                
                # Clean up calibration window
                cv2.destroyWindow("3D Calibration")
                
                # Calculate camera pose if we have enough points
                if len(calibration_points) >= 4:
                    print("\n🔧 Calculating camera pose...")
                    
                    # Load camera calibration data
                    if os.path.exists('calibration_data.npz'):
                        K, D = camera_processor.load_calibration_data('calibration_data.npz')
                        
                        # Extract object and image points
                        object_points = [point[0] for point in calibration_points]
                        image_points = [point[1] for point in calibration_points]
                        
                        # Calibrate camera pose
                        rvec, tvec = camera_processor.calibrate_3d_camera_pose(object_points, image_points, K, D)
                        
                        if rvec is not None and tvec is not None:
                            # Save calibration results
                            camera_processor.save_3d_calibration_data(
                                rvec, tvec, K, D, calibration_points, (1920, 1080)
                            )
                            
                            # Print calibration summary
                            print("\n📊 CALIBRATION SUMMARY:")
                            print(f"Points collected: {len(calibration_points)}")
                            print(f"Resolution: 1920x1080")
                            print(f"Camera matrix shape: {K.shape}")
                            print(f"Rotation vector: {rvec.flatten()}")
                            print(f"Translation vector: {tvec.flatten()}")
                            
                            # Test pixel-to-world conversion
                            if len(calibration_points) > 0:
                                test_point = calibration_points[0]
                                test_pixel = test_point[1]
                                test_world = test_point[0]
                                Z_known = test_world[2]
                                
                                calculated_world = camera_processor.pixel_to_world(
                                    test_pixel[0], test_pixel[1], Z_known, K, D, rvec, tvec
                                )
                                
                                error = np.linalg.norm(np.array(test_world) - calculated_world)
                                print(f"Test conversion error: {error:.3f} cm")
                        else:
                            print("❌ Camera pose calculation failed")
                    else:
                        print("❌ Camera calibration data not found. Run distortion calibration first.")
                else:
                    print("❌ Not enough points for calibration")
                
                # Switch back to original resolution
                print("Switching back to original resolution...")
                picam2.stop()
                time.sleep(1)
                config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
                picam2.configure(config)
                picam2.start()
                time.sleep(1)
                
                print("3D Gripper Real World Calibration finished.")
            elif key == ord('t'):  # Test 3D calibration
                print("\n🧪 Testing 3D calibration...")
                camera_processor.test_3d_calibration()
            elif key == ord('m'):
                manual_camera_controls(picam2)
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
