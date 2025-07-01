# ArmPi Mini Test 3D Calibration Guide

## Objective
To validate the accuracy of your 3D camera calibration by testing the pixel-to-world coordinate conversion and optionally performing physical arm movements to verify real-world accuracy.

## Prerequisites
* Completed 3D Pose Calibration (key `y`)
* Camera distortion calibration data available (`calibration_data.npz`)
* 3D calibration data available (`3d_camera_pose.npz`)
* Robotic arm properly connected and operational

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Access Test 3D Calibration (Key: `t`)
1. Press the `t` key to start the 3D calibration test.
2. The system will automatically load your calibration data and begin testing.

---

## Step 3: Understanding the Test Process
The test function performs several validation steps:

### Mathematical Validation
1. **Loads calibration data**: Reads your camera matrix, distortion coefficients, and 3D pose data.
2. **Tests coordinate conversion**: Converts known world coordinates to pixel coordinates and back.
3. **Calculates error**: Measures the difference between original and calculated coordinates.
4. **Reports accuracy**: Displays the average error in centimeters.

### Physical Validation (Optional)
1. **Arm movement test**: The system may move the arm to test positions.
2. **Real-world verification**: Compares calculated positions with actual arm positions.
3. **Accuracy assessment**: Reports how well the calibration works in practice.

---

## Step 4: Interpreting Results
The test will provide several metrics:

### Error Measurements
* **Test conversion error**: Average distance between original and calculated world coordinates (in cm)
* **Pixel-to-world accuracy**: How well the system converts camera pixels to real-world positions
* **World-to-pixel accuracy**: How well the system converts real-world positions to camera pixels

### Quality Assessment
* **Good calibration**: Error < 1.0 cm
* **Acceptable calibration**: Error < 2.0 cm
* **Poor calibration**: Error > 2.0 cm (consider re-calibrating)

---

## Step 5: What the Test Validates
The test function checks:

1. **Data integrity**: Ensures calibration files are valid and complete
2. **Mathematical accuracy**: Tests the coordinate transformation equations
3. **Physical accuracy**: Validates that the calibration works in real-world scenarios
4. **System consistency**: Verifies that all components work together correctly

---

## Step 6: Understanding the Output
You'll see output similar to:
```
🧪 Testing 3D calibration...
📊 Loading calibration data...
🔍 Testing coordinate conversions...
📈 Test conversion error: 0.85 cm
✅ Calibration validation complete
```

---

## Important Notes
* **Prerequisites**: You must have completed both distortion and 3D pose calibration before testing.
* **Error interpretation**: Lower error values indicate better calibration accuracy.
* **Physical testing**: The system may move the arm during testing - ensure clear workspace.
* **Re-calibration**: If error is high (>2.0 cm), consider re-running the 3D pose calibration.

---

## Troubleshooting
* **Missing calibration data**: Complete distortion and 3D pose calibration first.
* **High error values**: Re-run 3D pose calibration with more precise positioning.
* **Arm movement issues**: Check arm connections and power before testing.
* **Inconsistent results**: Ensure stable lighting and camera position.

---

## What This Test Does
The 3D calibration test:

1. **Validates mathematical accuracy**: Tests the coordinate transformation equations
2. **Measures real-world accuracy**: Compares calculated vs. actual positions
3. **Assesses calibration quality**: Provides quantitative error measurements
4. **Identifies issues**: Helps detect problems with the calibration process
5. **Confirms system readiness**: Verifies the system is ready for accurate operation

---

## When to Run This Test
Run the 3D calibration test:
* After completing 3D pose calibration
* When experiencing accuracy issues
* After moving or adjusting the camera
* Before using the system for precise tasks
* As part of regular system maintenance

---

## Expected Results
For a well-calibrated system:
* **Error < 1.0 cm**: Excellent calibration
* **Error 1.0-2.0 cm**: Good calibration
* **Error > 2.0 cm**: Consider re-calibration

The test provides confidence that your calibration is accurate and the system can reliably convert between camera pixels and real-world coordinates.

---

## File Dependencies
The test requires these files:
* `calibration_data.npz`: Camera matrix and distortion coefficients
* `3d_camera_pose.npz`: 3D camera pose and calibration points

If either file is missing, the test will fail and prompt you to complete the corresponding calibration first. 