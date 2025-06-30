# ArmPi Mini Calibration Guide

## Objective
To accurately calibrate the camera so the system can translate a pixel seen by the camera into a real-world (X, Y, Z) coordinate for the robotic arm.

---

## Required Items
- A printed checkerboard pattern.
- A calibration grid: a piece of paper with the points marked.
- 3cm blocks for height calibration.
- (Optional) A color reference chart.
- FYI: The arm uses **11 PWM units per degree**.

---

## Coordinate System Overview

Based on the official ArmPi mini documentation, the coordinate system is defined as follows:

### Angled View
This shows the arm's general posture.
![Angled View of Arm](/home/pi/ArmPi_mini/docs/images/arm_angled_view.png)

### Top-Down View
The `+X` axis runs **to the right** from the arm base.
The `+Y` axis runs **forward** (away from the user).
The origin `(0,0,0)` is the center of the base.

![Top-Down View of Arm](/home/pi/ArmPi_mini/docs/images/arm_top_view.png)

### Side View
This view shows the Z axis. The `+Z` axis points **up** from the table surface.

![Side View of Arm](/home/pi/ArmPi_mini/docs/images/arm_side_view.png)

### Arm Dimensions
This shows the arm's general dimensions.
![Dimensions of Arm](/home/pi/ArmPi_mini/docs/images/arm_dimensions.png)

---

## Key Setup Points
- Arm base is the **origin (0,0,0)**. ![Coorinate system](/home/pi/ArmPi_mini/docs/images/arm_coordinate_system.png)
- `+X`: right from the base
- `+Y`: forward from the base (into the table)
- `+Z`: upward from the table
- Ready position: `(x=6, y=0, z=10)`
- Place the calibration grid **in front of the arm**, aligned with the axes.

---

## Updated Top-Down Layout (ASCII Diagram)

```
(View from Above)

                              +Y (forward → table)
                               ↑
                               |
            (-10,20)         (0,20)         (10,20)
                ●--------------●--------------●
                |              |              |
            (-10,15)         (0,15)         (10,15)
                ●--------------●--------------●
                |              |              |
            (-10,10)         (0,10)         (10,10)
                ●--------------●--------------●
                |              |              |
            (-10, 5)         (0, 5)         (10, 5)
                ●--------------●--------------●
                               |
                       [Arm Base @ (0,0)]
                               |
                              -Y
                     ← -X             +X →
```

---

## Calibration Point Arrangement

### ✅ XY Plane Points (Z = 0)
To cover the full table reach:

| X   | Y   | Z   | Notes                  |
|-----|-----|-----|------------------------|
| -10 |  5  |  0  | Far left, close        |
|  0  |  5  |  0  | Center, close          |
| 10  |  5  |  0  | Far right, close       |
| -10 | 10  |  0  | Far left, mid          |
|  0  | 10  |  0  | Center, mid            |
| 10  | 10  |  0  | Far right, mid         |
| -10 | 15  |  0  | Far left, far          |
|  0  | 15  |  0  | Center, far            |
| 10  | 15  |  0  | Far right, far         |
| -10 | 20  |  0  | Far left, max reach    |
|  0  | 20  |  0  | Center, max reach      |
| 10  | 20  |  0  | Far right, max reach   |

### ✅ Z Stack Points (central)
To include vertical variation:

| X | Y  | Z   | Notes            |
|---|----|-----|------------------|
| 0 | 15 |  3  | 1 block high     |
| 0 | 15 |  6  | 2 blocks high    |
| 0 | 15 |  9  | 3 blocks high    |
| 0 | 15 | 12  | 4 blocks high    |

---

## Calibration Process

### Step 1: Start the Calibration Script
```bash
python3 imx477/lab_adjust.py
```
You’ll see two windows: **'Original'** (live feed). The mask window is hidden for now **'Mask'** (color detection).

---

### Step 2: Camera Distortion Calibration (Key: `d`)
- Hold checkerboard pattern to camera.
- Press `c` to capture (aim for 10–15 good angles).
- Press `q` to complete.
- Saves `calibration_data.npz` with `K` and `D` values.

---

### Step 3: Guided 3D Pose Calibration (Key: `y`)
1. Press `y` (arm moves to ready position).
2. Press `u` for undistorted view.
3. Jog arm to touch each calibration point:
   - Keys: `w/s` (Lift), `a/d` (Shoulder), `q/e` (Elbow), `z/c` (Base)
4. Press **spacebar** when gripper touches point.
5. Click gripper tip in camera window.
6. Press `q` to save point, `n` to retry.

After all points, system saves `3d_camera_pose.npz`.

---

### Step 4: Test the Calibration (Key: `t`)
- Loads `3d_camera_pose.npz`
- Displays conversion error in cm
- **< 1.0 cm = good calibration**

---

## Additional Controls

### Jog Mode (Key: `j`)
Use keyboard to move arm manually:
- `w/s`, `a/d`, `q/e`, `z/c` for arm control
- `f/g` for gripper
- `r`: Reset to ready position
- `j`: Exit jog mode

### Undistortion Toggle (Key: `u`)
Toggles camera undistortion view.

---

After completing these steps, your system will be fully calibrated to translate 2D pixel coordinates into accurate real-world 3D positions.
