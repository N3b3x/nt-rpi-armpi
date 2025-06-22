# ArmPi Mini Calibration Guide

## Objective
To accurately calibrate the camera so the system can translate a pixel seen by the camera into a real-world (X, Y, Z) coordinate for the robotic arm.

## Required Items
*   A printed checkerboard pattern.
*   A calibration grid: a piece of paper with the points marked.
*   3cm blocks for height calibration.
*   Its 11 PWM per degrees just an FYI
*   (Optional) A color reference chart.

## Coordinate System and Grid Layout

Based on your feedback and the CAD files, the layout and coordinate system should now be correct.

### Angled View
This shows the arm's general posture.
![Angled View of Arm](/home/pi/ArmPi_mini/docs/images/arm_angled_view.png)

### Top-Down View
This view shows the X and Y axes. The `+X` axis runs forward from the arm. The `+Y` axis runs to the left. The `(0,0,0)` origin is the center of the base.
![Top-Down View of Arm](/home/pi/ArmPi_mini/docs/images/arm_top_view.png)

### Side View
This view shows the Z axis. The `+Z` axis points up from the table surface.
![Side View of Arm](/home/pi/ArmPi_mini/docs/images/arm_side_view.png)

**Key Setup Points:**
- The arm's base is the **Origin (0,0,0)**.
- **`+X`** is **forward** from the base.
- **`+Y`** is **left** of the base.
- **`+Z`** is **up** from the table.
- The arm starts at its "ready position" of `(x=6, y=0, z=10)`.
- The calibration grid is placed on the table in front of the arm.

### Gripper and Camera Axis
To be perfectly clear about the orientation of the end-effector:
-   **Forward Axis**: The camera and gripper are mounted together and point in the same forward direction.
-   **Alignment**: When the arm is extended straight forward (base angle 0) and is not pitched up or down, this forward direction is parallel to the world's **+X axis**.
-   **Viewing Objects**: To see and pick up objects on the table, the entire end-effector assembly (camera and gripper) pitches downwards.

### Unified Text-Based Layout

Here is a clearer, top-down view of the workspace layout.

```
(View from Above)

                                           +X (forward)
                                                |
      +-----------------------------------------|-----------------------------------------------------+
      |                                         |                                                     |
      |                           (20,-5) ..... | ..... (20, 5)                     |
      |                                         |                                                     |
      |                                   (18, 0) <-- Stack blocks here                               |
      |                           (15,-5) ..... | ..... (15, 5)                                       |
      |                                         |                                                     |
      +-----------------------------------------|-----------------------------------------------------+
                                                ^
                                                | 
                                              < | > Gripper @ Ready Position (6,0,10)
                                                v
                        <---- (-Y) Right ---- [ARM BASE @ 0,0,0] ---- (+Y) Left ---->

```

**Ready Position Clarified:**
*   The gripper starts **physically located** at world coordinates `(x=6, y=0, z=10)`.
*   This means it is 6cm **forward** from the arm base and is centered on the X-axis.
*   In this starting pose, the arm is reaching straight forward.

**Simple Coordinate Examples:**
- `(15, 5, 0.5)` = 15cm FORWARD, 5cm LEFT, 0.5cm UP
- `(15, -5, 0.5)` = 15cm FORWARD, 5cm RIGHT, 0.5cm UP  
- `(18, 0, 3)` = 18cm FORWARD, CENTERED, 3cm UP
- `(6, 0, 10)` = 6cm FORWARD, CENTERED, 10cm UP (The Ready Position)

**Grid Positioning Instructions:**
1. **Place your calibration grid paper on the table surface** in front of the arm.
2. **Position the grid so that:**
   - The arm base is at coordinate (0,0,0).
   - The grid extends forward (positive X direction) from the arm.
3. **Mark these points on your grid paper:**
   - `(15, 5, 0.5)`
   - `(20, 5, 0.5)`
   - `(15, -5, 0.5)`
   - `(20, -5, 0.5)`
   - `(18, 0, 3)` (1 block high)
   - `(18, 0, 6)` (2 blocks high)
   - `(18, 0, 9)` (3 blocks high)
   - `(18, 0, 12)` (4 blocks high)

**Important Notes:**
- The `CALIBRATION_GRID_POINTS` list in the `lab_adjust.py` code defines the 8 points you need to touch with the gripper tip.

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
4.  Move the checkerboard to different positions and angles. At each position, press the `c` key to capture an image.
5.  Capture at least 10-15 good images.
6.  Press `q` to finish capturing.
7.  The script will process the images and save the camera matrix (`K`) and distortion coefficients (`D`) to `calibration_data.npz`.

---

## Step 3: Guided 3D Pose Calibration (Key: `y`)
This step maps the camera's 2D view to the arm's 3D world.

1.  Press `y`. The arm will move to its Ready Position.
2.  The script will display **"STEP 1: JOG ARM"** and show the first target coordinate (e.g., `(15, 5, 0.5)`).
3.  Using your marked paper grid, use the jog keys to move the arm until the gripper tip physically touches that exact point.
    *   **Jog Keys:** `w/s` (Lift), `a/d` (Shoulder), `q/e` (Elbow), `z/c` (Base).
    *   **Other Keys:** `u` (Toggle undistortion), `i` (Show help), `y` (Exit calibration).
4.  Once positioned, press **spacebar**.
5.  The display changes to **"STEP 2: CLICK PIXEL"**. Click on the live video where the gripper's tip is.
6.  Press `q` to confirm and save this point.
7.  If you made a mistake, press `n` to restart the process for the *same* point.
8.  Repeat for all points.
9.  After all points, the script calculates the camera's 3D pose and saves it to `3d_camera_pose.npz`.

---

## Step 4: Test the Calibration (Key: `t`)
This verifies accuracy.

1.  Press `t`.
2.  The script loads `3d_camera_pose.npz` and runs a test.
3.  It prints a "Test conversion error" in cm.
4.  **A good calibration error is well below 1.0 cm.** If it's high, redo Step 3 more precisely.

---

## Additional Features

### Jog Mode (Key: `j`)
- Test arm movements with keyboard controls.
- Keys: `w/s` (Lift), `a/d` (Shoulder), `q/e` (Elbow), `z/c` (Base), `f/g` (Gripper).
- Press `r` to reset to the ready position, `j` to exit.

### Undistortion Toggle (Key: `u`)
- Toggles the undistortion view on/off.

---
After completing these steps, your system will be fully calibrated.
