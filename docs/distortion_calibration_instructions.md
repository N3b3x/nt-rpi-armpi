# ArmPi Mini Distortion Calibration Guide

## Objective
To correct the "fisheye" effect and other lens distortions by calibrating the camera using a checkerboard pattern. This creates a camera matrix and distortion coefficients that can be used to undistort images.

## Required Items
* A printed checkerboard pattern (6x9 internal corners)
* The checkerboard should be rigid and flat
* Good lighting conditions
* At least 10-15 different positions and angles of the checkerboard

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Access Distortion Calibration (Key: `d`)
1. Press the `d` key to enter Distortion Calibration mode.
2. The system will check if you have existing calibration images.

---

## Step 3: Choose Image Capture Strategy
The system will ask if you want to:
* **Reuse existing images** (press `y`): If you have previously captured calibration images and want to use them again.
* **Recapture new images** (press `n`): To capture a fresh set of calibration images (recommended for first-time calibration).

---

## Step 4: Capture Calibration Images
If recapturing images:
1. The camera will switch to high resolution (1920x1080) for better accuracy.
2. A window titled "Checkerboard Capture" will appear.
3. Hold the checkerboard pattern in front of the camera.
4. **Move the checkerboard to different positions and angles:**
   - Top, bottom, left, right of the camera view
   - Tilted at various angles
   - Different distances from the camera
   - Rotated orientations
5. Press `c` to capture an image at each position.
6. **Capture at least 10-15 good images** from different angles and positions.
7. Press `d` or `q` to finish capturing when you have enough images.

---

## Step 5: Processing and Completion
1. The system will automatically process all captured images.
2. It will detect the checkerboard corners and calculate:
   - Camera matrix (K)
   - Distortion coefficients (D)
3. Results are saved to `calibration_data.npz`.
4. You'll see "✅ Distortion calibration complete."

---

## Important Notes
* **Checkerboard quality**: Use a high-quality, rigid checkerboard pattern.
* **Coverage**: Capture images from all areas of the camera view for best results.
* **Lighting**: Ensure the checkerboard is well-lit and clearly visible.
* **Movement**: Move the checkerboard slowly and steadily to avoid motion blur.
* **Resolution**: The system automatically switches to high resolution for better accuracy.

---

## Troubleshooting
* **Checkerboard not detected**: Ensure the pattern is fully visible and well-lit.
* **Poor calibration**: Capture more images from different angles and positions.
* **Blurry images**: Hold the checkerboard steady when capturing.
* **Insufficient images**: Capture at least 10-15 images for good calibration.

---

## What This Calibration Does
The distortion calibration:
1. Captures multiple images of a checkerboard from different angles
2. Detects the internal corners of the checkerboard pattern
3. Calculates the camera's intrinsic parameters (focal length, principal point)
4. Determines distortion coefficients to correct lens aberrations
5. Saves the calibration data for use in undistorting images

After completing this calibration, you can:
* Toggle undistortion on/off using the `u` key in the main interface
* Use undistorted images for more accurate color detection and 3D calibration
* Improve the overall accuracy of the vision system

---

## File Output
The calibration creates:
* `calibration_data.npz`: Contains the camera matrix (K) and distortion coefficients (D)
* `calib_images/`: Directory containing all captured calibration images

This calibration is essential for accurate 3D pose calibration and should be completed before attempting 3D calibration. 