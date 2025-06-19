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
from color_palletizing.arm_controller import ArmController, SERVO_GRIPPER, SERVO_ELBOW, SERVO_SHOULDER, SERVO_LIFT, SERVO_BASE
from color_palletizing.camera_processor import CameraProcessor
import math

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
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    arm_controller = ArmController()  # Create arm controller instance
    camera_processor = CameraProcessor()  # Create camera processor instance

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
                    K, D = camera_processor.load_calibration_data('calibration_data.npz')
            elif key == ord('j'):  # New option for jog mode
                print("\n🔍 Starting with jogging controls...")
                print("Use keyboard controls to position the arm:")
                print("  [f/g] - Gripper open/close")
                print("  [q/e] - Elbow up/down") 
                print("  [a/d] - Shoulder forward/back")
                print("  [w/s] - Lift up/down")
                print("  [z/c] - Base rotate left/right")
                print("  [r] - Reset to center position")
                print("  [j] - Exit Jog mode")
                
                # Initialize arm to vertical position (like WonderPi)
                arm_controller.current_lift_angle = 0
                arm_controller.current_shoulder_angle = 0
                arm_controller.current_elbow_angle = 0
                arm_controller.current_base_angle = 0
                arm_controller.current_gripper_pos = 1500
                
                # Move to vertical position (like WonderPi - all servos to 1500 with deviation)
                deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
                data = [
                    [SERVO_GRIPPER, 1500 + deviation_data['1']],  # Gripper
                    [SERVO_ELBOW,   1500 + deviation_data['3']],  # Elbow
                    [SERVO_SHOULDER,1500 + deviation_data['4']],  # Shoulder
                    [SERVO_LIFT,    1500 + deviation_data['5']],  # Lift
                    [SERVO_BASE,    1500 + deviation_data['6']]   # Base
                ]
                arm_controller.board.pwm_servo_set_position(1.5, data)
                time.sleep(1.5)  # Wait for movement to complete
                
                jog_mode = True
                while jog_mode:
                    frame_jog = picam2.capture_array()
                    if frame_jog is not None:
                        frame_bgr = cv2.cvtColor(frame_jog, cv2.COLOR_RGB2BGR)
                        
                        # Draw jogging instructions on frame
                        instructions = [
                            "Jog Controls:",
                            "[f/g] Gripper open/close",
                            "[q/e] Elbow up/down",
                            "[a/d] Shoulder forward/back",
                            "[w/s] Lift up/down",
                            "[z/c] Base rotate left/right",
                            "[r] Reset to center",
                            "[x] Exit Jog Mode"
                        ]
                        for idx, text in enumerate(instructions):
                            cv2.putText(frame_bgr, text, (10, 30 + 25 * idx),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
                        # Show current joint angles
                        angle_info = [
                            f"Gripper: {arm_controller.current_gripper_pos}",
                            f"Elbow: {arm_controller.current_elbow_angle:.1f} deg",
                            f"Shoulder: {arm_controller.current_shoulder_angle:.1f} deg",
                            f"Lift: {arm_controller.current_lift_angle:.1f} deg",
                            f"Base: {arm_controller.current_base_angle:.1f} deg"
                        ]
                        for idx, text in enumerate(angle_info):
                            cv2.putText(frame_bgr, text, (10, frame_bgr.shape[0] - 100 + 20 * idx),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        cv2.imshow("Jog Mode", frame_bgr)
                        
                        jog_key = cv2.waitKey(1) & 0xFF
                        if jog_key == ord('x'):
                            print("Exiting Jog Mode...")
                            jog_mode = False
                        elif jog_key == ord('f'):
                            arm_controller.move_gripper(1)
                        elif jog_key == ord('g'):
                            arm_controller.move_gripper(-1)
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
                            # Reset to vertical position (like WonderPi)
                            arm_controller.current_lift_angle = 0
                            arm_controller.current_shoulder_angle = 0
                            arm_controller.current_elbow_angle = 0
                            arm_controller.current_base_angle = 0
                            arm_controller.current_gripper_pos = 1500
                            deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
                            data = [
                                [SERVO_GRIPPER, 1500 + deviation_data['1']],  # Gripper
                                [SERVO_ELBOW,   1500 + deviation_data['3']],  # Elbow
                                [SERVO_SHOULDER,1500 + deviation_data['4']],  # Shoulder
                                [SERVO_LIFT,    1500 + deviation_data['5']],  # Lift
                                [SERVO_BASE,    1500 + deviation_data['6']]   # Base
                            ]
                            arm_controller.board.pwm_servo_set_position(1.5, data)
                            time.sleep(1.5)  # Wait for movement to complete
                    else:
                        time.sleep(0.01)
                
                cv2.destroyWindow("Jog Mode")
                print("Jogging mode finished.")
            elif key == ord('y'):  # 3D Gripper Real World Calibration
                print("\n🎯 Starting 3D Gripper Real World Calibration...")
                print("This mode will use higher resolution for better accuracy.")
                print("Steps:")
                print("1. Jog arm to touch a known point")
                print("2. Press [space] to record arm position")
                print("3. Click the corresponding pixel in the image")
                print("4. Repeat for at least 6 points")
                print("5. Press [q] to finish and calculate camera pose")
                
                # Switch to higher resolution for better accuracy
                print("Switching to higher resolution mode...")
                picam2.stop()
                time.sleep(1)
                
                # Configure for higher resolution (IMX477 can do 4056x3040)
                # Using 1920x1080 for good balance of accuracy and performance
                config = picam2.create_preview_configuration(
                    main={"size": (1920, 1080)},
                    controls={"FrameDurationLimits": (33333, 33333)}  # 30 FPS
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(2)

                # Initialize calibration data
                calibration_points = []
                clicked_pixel = None
                
                # Set up mouse callback
                cv2.namedWindow("3D Calibration - Click Pixel")
                cv2.setMouseCallback("3D Calibration - Click Pixel", camera_processor.mouse_callback)
                
                # Initialize arm to vertical position
                arm_controller.current_lift_angle = 0
                arm_controller.current_shoulder_angle = 0
                arm_controller.current_elbow_angle = 0
                arm_controller.current_base_angle = 0
                arm_controller.current_gripper_pos = 1500
                
                deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
                data = [
                    [SERVO_GRIPPER, 1500 + deviation_data['1']],
                    [SERVO_ELBOW,   1500 + deviation_data['3']],
                    [SERVO_SHOULDER,1500 + deviation_data['4']],
                    [SERVO_LIFT,    1500 + deviation_data['5']],
                    [SERVO_BASE,    1500 + deviation_data['6']]
                ]
                arm_controller.board.pwm_servo_set_position(1.5, data)
                time.sleep(1.5)
                
                point_count = 0
                calib_mode = True
                
                while calib_mode:
                    frame_calib = picam2.capture_array()
                    if frame_calib is not None:
                        frame_bgr = cv2.cvtColor(frame_calib, cv2.COLOR_RGB2BGR)
                        
                        # Draw calibration instructions
                        instructions = [
                            f"3D GRIPPER CALIBRATION - Point {point_count + 1}/6",
                            "",
                            "JOG CONTROLS:",
                            "[f/g] Gripper open/close",
                            "[q/e] Elbow up/down",
                            "[a/d] Shoulder forward/back",
                            "[w/s] Lift up/down",
                            "[z/c] Base rotate left/right",
                            "[r] Reset to center",
                            "",
                            "CALIBRATION:",
                            "[space] Record arm position",
                            "[q] Finish calibration",
                            "",
                            "After pressing [space], click the pixel where gripper touches"
                        ]
                        
                        for idx, text in enumerate(instructions):
                            cv2.putText(frame_bgr, text, (10, 30 + 25 * idx),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        # Show current joint angles and estimated position
                        arm_pos = arm_controller.get_arm_position()
                        angle_info = [
                            f"Arm Pos: ({arm_pos[0]:.1f}, {arm_pos[1]:.1f}, {arm_pos[2]:.1f}) cm",
                            f"Gripper: {arm_controller.current_gripper_pos}",
                            f"Elbow: {arm_controller.current_elbow_angle:.1f}°",
                            f"Shoulder: {arm_controller.current_shoulder_angle:.1f}°",
                            f"Lift: {arm_controller.current_lift_angle:.1f}°",
                            f"Base: {arm_controller.current_base_angle:.1f}°"
                        ]
                        
                        for idx, text in enumerate(angle_info):
                            cv2.putText(frame_bgr, text, (10, frame_bgr.shape[0] - 120 + 20 * idx),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        # Draw collected points on image
                        for i, (world_pos, pixel_pos) in enumerate(calibration_points):
                            cv2.circle(frame_bgr, pixel_pos, 10, (0, 255, 0), 2)
                            cv2.putText(frame_bgr, f"P{i+1}", 
                                        (pixel_pos[0] + 15, pixel_pos[1] - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        cv2.imshow("3D Calibration - Click Pixel", frame_bgr)
                        
                        calib_key = cv2.waitKey(1) & 0xFF
                        
                        if calib_key == ord('q'):
                            if len(calibration_points) >= 4:
                                print(f"✅ Finishing calibration with {len(calibration_points)} points")
                                calib_mode = False
                            else:
                                print("❌ Need at least 4 points for calibration")
                        elif calib_key == ord(' '):  # Space bar
                            if camera_processor.clicked_pixel is None:
                                print("⚠️  Please click a pixel first, then press [space]")
                            else:
                                arm_pos = arm_controller.get_arm_position()
                                clicked_pixel = camera_processor.clicked_pixel
                                calibration_points.append((arm_pos, clicked_pixel))
                                point_count += 1
                                camera_processor.clicked_pixel = None
                                print(f"✅ Point {point_count} recorded: Arm({arm_pos[0]:.1f}, {arm_pos[1]:.1f}, {arm_pos[2]:.1f}) -> Pixel{clicked_pixel}")
                        elif calib_key == ord('f'):
                            arm_controller.move_gripper(1)
                        elif calib_key == ord('g'):
                            arm_controller.move_gripper(-1)
                        elif calib_key == ord('q'):
                            arm_controller.move_elbow(2)
                        elif calib_key == ord('e'):
                            arm_controller.move_elbow(-2)
                        elif calib_key == ord('a'):
                            arm_controller.move_shoulder(-2)
                        elif calib_key == ord('d'):
                            arm_controller.move_shoulder(2)
                        elif calib_key == ord('w'):
                            arm_controller.move_lift(2)
                        elif calib_key == ord('s'):
                            arm_controller.move_lift(-2)
                        elif calib_key == ord('z'):
                            arm_controller.move_base(-2)
                        elif calib_key == ord('c'):
                            arm_controller.move_base(2)
                        elif calib_key == ord('r'):
                            # Reset to vertical position
                            arm_controller.current_lift_angle = 0
                            arm_controller.current_shoulder_angle = 0
                            arm_controller.current_elbow_angle = 0
                            arm_controller.current_base_angle = 0
                            arm_controller.current_gripper_pos = 1500
                            deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
                            data = [
                                [SERVO_GRIPPER, 1500 + deviation_data['1']],
                                [SERVO_ELBOW,   1500 + deviation_data['3']],
                                [SERVO_SHOULDER,1500 + deviation_data['4']],
                                [SERVO_LIFT,    1500 + deviation_data['5']],
                                [SERVO_BASE,    1500 + deviation_data['6']]
                            ]
                            arm_controller.board.pwm_servo_set_position(1.5, data)
                            time.sleep(1.5)
                    else:
                        time.sleep(0.01)
                
                # Clean up calibration window
                cv2.destroyWindow("3D Calibration - Click Pixel")
                cv2.destroyAllWindows()
                
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
                config = picam2.create_preview_configuration()
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
