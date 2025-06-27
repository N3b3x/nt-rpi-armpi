#!/usr/bin/python3
# coding=utf8
import os
import shutil
import time
import cv2
import numpy as np
import glob
import yaml
from Camera import Camera

def capture_reference_image(frame, save_path='reference_image.jpg'):
    """
    Save the provided frame (from the main script's live camera feed) as the reference image for calibration.
    The frame should be in RGB order (as returned by Picamera2).
    """
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(save_path, frame_bgr)
    print(f"✅ Reference image saved to {save_path}")

def calibrate_lab_ranges(reference_image_path='reference_image.jpg', yaml_output_path='lab_ranges.yaml', lab_tolerance=10):
    image = cv2.imread(reference_image_path)
    if image is None:
        print("❌ Could not load reference image!")
        return

    image_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    def get_average_lab(x, y, size=5):
        x1, y1 = max(0, x - size), max(0, y - size)
        x2, y2 = min(image_lab.shape[1], x + size), min(image_lab.shape[0], y + size)
        region = image_lab[y1:y2, x1:x2]
        mean_lab = np.mean(region.reshape(-1, 3), axis=0)
        return mean_lab.astype(int)

    points = []
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            avg_lab = get_average_lab(x, y)
            points.append(avg_lab)
            print(f"Clicked at ({x}, {y}), Average LAB: {avg_lab}")

    cv2.namedWindow("Reference Image - Click Colors")
    cv2.setMouseCallback("Reference Image - Click Colors", click_event)

    color_order = ['red', 'green', 'blue', 'black', 'white']
    print("🟦 Click on the following patches in order: RED, GREEN, BLUE, BLACK, WHITE. Press 'q' to abort.")
    while True:
        display_image = image.copy()
        cv2.putText(display_image, "Click: RED, GREEN, BLUE, BLACK, WHITE", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        for idx, color in enumerate(color_order):
            if len(points) > idx:
                cv2.putText(display_image, f"{color.upper()} set", (10, 60 + idx*30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Reference Image - Click Colors", display_image)

        if len(points) == 5:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Calibration aborted.")
            cv2.destroyAllWindows()
            return

    cv2.destroyAllWindows()

    lab_ranges = {}
    for i, color in enumerate(color_order):
        mean_lab = points[i]
        if isinstance(mean_lab, np.ndarray):
            mean_lab = mean_lab.tolist()
        min_lab = [max(0, mean_lab[j] - lab_tolerance) for j in range(3)]
        max_lab = [min(255, mean_lab[j] + lab_tolerance) for j in range(3)]
        lab_ranges[color] = {'min': min_lab, 'max': max_lab}

    with open(yaml_output_path, 'w') as f:
        yaml.dump(lab_ranges, f, default_flow_style=False)
    print(f"✅ LAB ranges saved and updated at: {yaml_output_path}")
        
def get_average_brightness_lab(frame_lab):
    l_channel = frame_lab[:, :, 0]
    avg_l = np.mean(l_channel)
    return avg_l

def check_and_auto_calibrate(frame_lab, previous_avg_l, threshold, reference_image_path, yaml_output_path):
    current_avg_l = get_average_brightness_lab(frame_lab)
    if previous_avg_l is not None and abs(current_avg_l - previous_avg_l) > threshold:
        print("⚠️ Detected lighting change! Auto-calibrating...")
        calibrate_lab_ranges(reference_image_path, yaml_output_path)
        return current_avg_l
    return current_avg_l

def distortion_calibration_with_existing_camera(camera, checkerboard=(7, 7), save_path='calibration_data.npz'):
    image_dir = 'calib_images'
    if os.path.exists(image_dir):
        shutil.rmtree(image_dir)
    os.makedirs(image_dir, exist_ok=True)

    print("\n🔍 Please hold up the checkerboard for distortion calibration.")
    print("Move the checkerboard around slightly and take at least 10 images from different angles.")
    print("Press 'c' to capture calibration images, 'q' to finish capture.")

    img_counter = 0

    while True:
        frame_cb = camera.get_frame()
        if frame_cb is not None:
            #frame_bgr = frame_cb[..., ::-1]
            cv2.putText(frame_cb, "Move checkerboard. Press 'c' to capture, 'q' to finish.",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Checkerboard Capture", frame_cb)

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

    # Start the actual calibration
    calibrate_camera(image_dir=image_dir, checkerboard=checkerboard, save_path=save_path)


def calibrate_camera(image_dir='calib_images', checkerboard=(6, 9), square_size=20.0, save_path='calibration_data.npz'):
    import glob
    import numpy as np

    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)
    objp = objp * square_size # Scale by square size

    objpoints = []  # 3d real world points
    imgpoints = []  # 2d image points

    images = glob.glob(os.path.join(image_dir, '*.jpg'))
    print(f"🔍 Found {len(images)} images for calibration.")

    if len(images) < 5:
        print("❌ Not enough images for reliable calibration. Please capture at least 5.")
        return

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    good_images = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"⚠️ Could not load {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Try CLAHE enhanced image
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray_clahe = clahe.apply(gray)

        found = False
        for attempt, g in [('CLAHE', gray_clahe), ('Raw', gray)]:
            ret, corners = cv2.findChessboardCornersSB(g, checkerboard, None)
        if not ret:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            ret, corners = cv2.findChessboardCorners(g, checkerboard, flags)
        if ret:
            print(f"✅ {attempt}: Found corners in {fname}")
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, checkerboard, corners2, ret)
            cv2.imshow('Corners', img)
            cv2.waitKey(300)
            good_images += 1
            found = True
            break
        if not found:
            print(f"❌ No corners found in {fname}")

    cv2.destroyAllWindows()

    if good_images < 5:
        print("❌ Too few valid images. Calibration aborted.")
        return

    # Use a different check for fisheye calibration
    if len(objpoints) == 0:
        print("❌ No valid points found for calibration.")
        return

    # Perform fisheye calibration
    try:
        # We need to reshape the arrays for the fisheye function
        objpoints_arr = np.expand_dims(np.asarray(objpoints), -2).astype(np.float32)
        imgpoints_arr = np.asarray(imgpoints).astype(np.float32)
        
        # Flags for fisheye calibration
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
        
        ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints_arr, imgpoints_arr, gray.shape[::-1], None, None, flags=flags
        )
        
        # Save calibration data
        np.savez(save_path, K=K, D=D, rvecs=rvecs, tvecs=tvecs, square_size=square_size)
            
        print(f"✅ Fisheye calibration successful. Saved to {save_path}")
        print(f"✅ Camera Matrix (K):\n{K}")
        print(f"✅ Distortion Coefficients (D):\n{D}")

    except Exception as e:
        print(f"❌ An error occurred during fisheye calibration: {e}")

def undistort_frame(frame, K, D):
    """Apply fisheye undistortion to a frame."""
    # Use the fisheye undistortion function
    # We pass K as the new camera matrix to keep the same scaling
    return cv2.fisheye.undistortImage(frame, K, D, Knew=K)

def set_controls(picam2,
                awb_enable=False,
                awb_mode=0,
                colour_gains=(1.5, 1.2),
                ae_enable=False,
                exposure_time=10000,
                analogue_gain=1.0,
                sharpness=8.0,
                contrast=1.0,
                saturation=1.0,
                brightness=0.0):
    """
    Apply manual camera settings to a Picamera2 instance.
    """
    controls = {
        "AwbEnable": awb_enable,
        "AwbMode": awb_mode,
        "ColourGains": colour_gains,
        "AeEnable": ae_enable,
        "ExposureTime": exposure_time,
        "AnalogueGain": analogue_gain,
        "Sharpness": sharpness,
        "Contrast": contrast,
        "Saturation": saturation,
        "Brightness": brightness
    }
    # Only set controls that are actually supported by this camera
    available = picam2.camera_controls
    filtered = {k: v for k, v in controls.items() if k in available}
    picam2.set_controls(filtered)
    print("Camera controls set:", filtered)

def load_calibration_data(path='calibration_data.npz'):
    data = np.load(path)
    K = data['K']
    D = data['D']
    return K, D

def calibrate_coordinates(get_frame, checkerboard=(6, 9), square_size=20.0, save_path='coordinate_calibration.npz', arm_controller=None):
    """
    Calibrate camera coordinates using a checkerboard pattern.
    This establishes the mapping between pixel coordinates and real-world coordinates.
    Args:
        get_frame: function that returns a BGR frame
        checkerboard: Tuple of (rows, cols) in the checkerboard
        square_size: Size of each square in mm
        save_path: Where to save the calibration data
        arm_controller: ArmController instance for jogging axes (optional)
    """
    print("\n🔍 Please place the checkerboard flat on the table.")
    print("The checkerboard should be visible in the camera view.")
    print("Press 'c' to capture the calibration image, 'q' to cancel.")
    print("Use jog keys to move the arm for better alignment.")

    # Jog step in degrees
    JOG_STEP = 2

    # On-screen jog instructions
    jog_instructions = [
        "Jog Controls:",
        "Base: z(+)/x(-)  Lift: w(+)/s(-)",
        "Shoulder: e(+)/d(-)  Elbow: r(+)/f(-)",
        "[c]=Capture  [q]=Quit"
    ]

    while True:
        frame = get_frame()
        if frame is not None:
            # Draw jog instructions
            for idx, text in enumerate(jog_instructions):
                cv2.putText(frame, text, (10, 30 + 25 * idx),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, "Align checkerboard, then press 'c' to capture", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Coordinate Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Find checkerboard corners
                ret, corners = cv2.findChessboardCorners(gray, checkerboard, None)
                if ret:
                    # Refine corner positions
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    # Draw corners
                    cv2.drawChessboardCorners(frame, checkerboard, corners2, ret)
                    cv2.imshow("Calibration Image", frame)
                    cv2.waitKey(1000)
                    # Calculate real-world coordinates of corners
                    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
                    objp[:, :2] = np.mgrid[0:checkerboard[1], 0:checkerboard[0]].T.reshape(-1, 2)
                    objp *= square_size  # scale to actual size in mm
                    # Get camera matrix and distortion coefficients
                    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                        [objp], [corners2], gray.shape[::-1], None, None
                    )
                    # Save calibration data
                    np.savez(save_path,
                            camera_matrix=mtx,
                            dist_coeffs=dist,
                            rvecs=rvecs,
                            tvecs=tvecs,
                            square_size=square_size)
                    print(f"✅ Coordinate calibration complete. Data saved to {save_path}")
                    print(f"🔍 Camera matrix:\n{mtx}")
                    break
                else:
                    print("❌ Could not find checkerboard corners. Please try again.")
            elif key == ord('q'):
                print("Calibration cancelled.")
                break
            # Jog controls
            elif arm_controller is not None:
                if key == ord('z'):
                    # Base +
                    arm_controller.rotate_base(JOG_STEP)
                elif key == ord('x'):
                    # Base -
                    arm_controller.rotate_base(-JOG_STEP)
                elif key == ord('w') or key == ord('s'):
                    # Lift
                    direction = JOG_STEP if key == ord('w') else -JOG_STEP
                    arm_controller.move_lift(direction)
                elif key == ord('e') or key == ord('d'):
                    # Shoulder
                    direction = JOG_STEP if key == ord('e') else -JOG_STEP
                    arm_controller.move_shoulder(direction)
                elif key == ord('r') or key == ord('f'):
                    # Elbow
                    direction = JOG_STEP if key == ord('r') else -JOG_STEP
                    arm_controller.move_elbow(direction)
        else:
            time.sleep(0.1)
    cv2.destroyAllWindows()

def pixel_to_world(pixel_coords, camera_matrix, rvecs, tvecs):
    """
    Convert pixel coordinates to real-world coordinates using camera calibration.
    
    Args:
        pixel_coords: (x, y) pixel coordinates
        camera_matrix: Camera calibration matrix
        rvecs: Rotation vectors from calibration
        tvecs: Translation vectors from calibration
    
    Returns:
        (x, y, z) real-world coordinates in mm
    """
    # Convert pixel coordinates to normalized image coordinates
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    x = (pixel_coords[0] - cx) / fx
    y = (pixel_coords[1] - cy) / fy
    
    # Convert to real-world coordinates using rotation and translation
    R, _ = cv2.Rodrigues(rvecs[0])
    t = tvecs[0]
    
    # Solve for Z (assuming Z is constant, e.g., table height)
    Z = t[2]  # This should be the height of the checkerboard from the camera
    
    # Calculate real-world coordinates
    X = (x * Z - t[0]) / R[0, 0]
    Y = (y * Z - t[1]) / R[1, 1]
    
    return (X, Y, Z)

def test_3d_calibration_with_physical_validation(arm_controller=None):
    """
    Comprehensive test of 3D calibration including both mathematical validation
    and physical movement testing.
    
    Args:
        arm_controller: ArmController instance for physical movement testing (optional)
    """
    import os
    import numpy as np
    
    # Load 3D calibration data
    if not os.path.exists('3d_camera_pose.npz'):
        print("❌ 3D calibration file not found: 3d_camera_pose.npz")
        print("   Please run 3D calibration first (press 'y' in lab_adjust.py)")
        return
    
    data = np.load('3d_camera_pose.npz', allow_pickle=True)
    calib_data = {
        'rvec': data['rvec'],
        'tvec': data['tvec'],
        'camera_matrix': data['camera_matrix'],
        'dist_coeffs': data['dist_coeffs'],
        'calibration_points': data['calibration_points'],
        'resolution': tuple(data['resolution'])
    }
    
    print("\n🧪 Testing 3D calibration...")
    print(f"Resolution: {calib_data['resolution']}")
    print(f"Calibration points: {len(calib_data['calibration_points'])}")
    
    # Test with the first calibration point
    test_point = calib_data['calibration_points'][0]
    test_world = test_point[0]
    test_pixel = test_point[1]
    Z_known = test_world[2]
    
    # Import the pixel_to_world function from camera_processor
    try:
        from color_palletizing.camera_processor import CameraProcessor
        camera_processor = CameraProcessor()
        calculated_world = camera_processor.pixel_to_world(
            test_pixel[0], test_pixel[1], Z_known,
            calib_data['camera_matrix'], calib_data['dist_coeffs'],
            calib_data['rvec'], calib_data['tvec']
        )
    except ImportError:
        print("❌ Could not import CameraProcessor for mathematical test")
        return
    
    error = np.linalg.norm(np.array(test_world) - calculated_world)
    print(f"\n📊 Mathematical Validation:")
    print(f"Test point: World({test_world[0]:.1f}, {test_world[1]:.1f}, {test_world[2]:.1f})")
    print(f"Test pixel: ({test_pixel[0]}, {test_pixel[1]})")
    print(f"Calculated: ({calculated_world[0]:.1f}, {calculated_world[1]:.1f}, {calculated_world[2]:.1f})")
    print(f"Error: {error:.3f} cm")
    
    if error < 1.0:
        print("✅ Mathematical calibration test passed!")
    else:
        print("⚠️  Mathematical calibration test shows high error - may need recalibration")
    
    # Physical movement test (if arm controller is provided)
    if arm_controller is not None:
        print(f"\n🤖 Physical Movement Test:")
        print(f"Moving arm to calculated position: ({calculated_world[0]:.1f}, {calculated_world[1]:.1f}, {calculated_world[2]:.1f})")
        
        # Move to a safe height first
        safe_height = max(calculated_world[2] + 5, 10)  # 5cm above target or minimum 10cm
        print(f"Moving to safe height: {safe_height:.1f} cm")
        
        try:
            # Move to safe position above target
            success = arm_controller.move_to_position(
                (calculated_world[0], calculated_world[1], safe_height), 
                pitch=-45, 
                movetime=2000
            )
            
            if success:
                print("✅ Successfully moved to safe position above target")
                
                # Ask user if they want to lower to the actual target
                print("\nDo you want to lower the arm to the actual target position?")
                print("This will move the gripper to the calculated coordinates.")
                print("Press 'y' to continue, any other key to skip...")
                
                import sys
                import tty
                import termios
                
                # Get a single character input
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                if ch.lower() == 'y':
                    print(f"Lowering to target position: ({calculated_world[0]:.1f}, {calculated_world[1]:.1f}, {calculated_world[2]:.1f})")
                    
                    # Move to actual target position
                    success = arm_controller.move_to_position(
                        (calculated_world[0], calculated_world[1], calculated_world[2]), 
                        pitch=-60, 
                        movetime=1500
                    )
                    
                    if success:
                        print("✅ Successfully moved to target position!")
                        print("\n🔍 Physical Validation:")
                        print("1. Check if the gripper tip is close to the original calibration point")
                        print("2. If the gripper is far from the expected position, the calibration needs adjustment")
                        print("3. Press any key to return to safe position...")
                        
                        # Wait for user input
                        input()
                        
                        # Return to safe position
                        print("Returning to safe position...")
                        arm_controller.move_to_position(
                            (calculated_world[0], calculated_world[1], safe_height), 
                            pitch=-45, 
                            movetime=1500
                        )
                    else:
                        print("❌ Failed to move to target position - position may be unreachable")
                else:
                    print("Physical movement test skipped")
            else:
                print("❌ Failed to move to safe position - check arm limits")
                
        except Exception as e:
            print(f"❌ Error during physical movement test: {e}")
    else:
        print("\n💡 To test physical movement, run this function with an ArmController instance")
        print("   Example: test_3d_calibration_with_physical_validation(arm_controller)")
    
    print("\n📋 Test Summary:")
    print(f"Mathematical error: {error:.3f} cm")
    if error < 0.5:
        print("🎯 Excellent calibration!")
    elif error < 1.0:
        print("✅ Good calibration")
    elif error < 2.0:
        print("⚠️  Acceptable calibration, but could be improved")
    else:
        print("❌ Poor calibration - recommend recalibration")
