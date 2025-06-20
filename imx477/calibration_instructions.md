
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
