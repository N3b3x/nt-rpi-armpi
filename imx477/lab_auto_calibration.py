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

    # Setup object points in real world space
    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard[1], 0:checkerboard[0]].T.reshape(-1, 2)
    objp *= square_size  # scale to actual size in mm

    objpoints = []  # 3d real world points
    imgpoints = []  # 2d image points

    images = glob.glob(os.path.join(image_dir, '*.jpg'))
    print(f"Found {len(images)} images to process for calibration.")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply CLAHE for contrast enhancement
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray_clahe = clahe.apply(gray)

        # Try using findChessboardCornersSB
        ret, corners = cv2.findChessboardCornersSB(gray_clahe, checkerboard, None)

        if not ret:
            # fallback to standard checkerboard detection
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            ret, corners = cv2.findChessboardCorners(gray_clahe, checkerboard, flags)

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, checkerboard, corners2, ret)
            cv2.imshow('Corners', img)
            cv2.waitKey(300)
        else:
            print(f"⚠️ Checkerboard not detected in {fname}")

    cv2.destroyAllWindows()

    if len(objpoints) == 0:
        print("❌ No checkerboard corners were detected in any images. Calibration aborted.")
        return

    # Calibrate the camera
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savez(save_path, K=K, D=D, rvecs=rvecs, tvecs=tvecs, square_size=square_size)
    print(f"✅ Distortion calibration complete. Calibration data saved to {save_path}")
    print(f"🔍 Camera matrix:\n{K}\n🔍 Distortion coefficients:\n{D}")

def undistort_frame(frame, K, D):
    h, w = frame.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    return cv2.undistort(frame, K, D, None, new_K)

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
