# ArmPi Mini Color Calibration Guide

## Objective
To calibrate the color detection system by capturing reference colors from a ColorChecker chart and setting up LAB color space ranges for accurate color recognition.

## Required Items
* A ColorChecker chart (or similar color reference card)
* Good lighting conditions
* The ColorChecker should be clearly visible to the camera

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Access Color Calibration (Key: `c`)
1. Press the `c` key to enter Color Calibration mode.
2. A new window titled "Color Calibration - Capture Reference" will appear.

---

## Step 3: Capture Reference Image
1. Hold up the ColorChecker chart clearly in front of the camera.
2. Ensure the chart is well-lit and all colors are clearly visible.
3. You can toggle undistortion on/off using the `u` key if needed.
4. Press `s` to capture the reference image.
5. The image will be saved as `reference_image.jpg`.
6. Press `c` or `q` to cancel if needed.

---

## Step 4: Select Color Patches
After capturing the reference image:
1. A new window titled "Reference Image - Click Colors" will appear.
2. You need to click on 5 specific colors in this order:
   - **RED**
   - **GREEN** 
   - **BLUE**
   - **BLACK**
   - **WHITE**
3. Click on each color patch in the exact order listed above.
4. The system will display "RED set", "GREEN set", etc. as you click each color.
5. Continue until all 5 colors are selected.

---

## Step 5: Complete Calibration
1. Once all 5 colors are clicked, the calibration will automatically complete.
2. The system will update the LAB color ranges based on your selections.
3. You'll see the message "✅ Calibration complete. LAB ranges updated!"
4. The windows will close automatically.

---

## Important Notes
* **Order matters**: You must click the colors in the exact order: Red, Green, Blue, Black, White.
* **Lighting**: Ensure consistent lighting during both capture and color selection.
* **Undistortion**: You can toggle undistortion on/off during reference capture if you have camera calibration data.
* **Accuracy**: Click precisely on the center of each color patch for best results.

---

## Troubleshooting
* **Can't see colors clearly**: Improve lighting or move the ColorChecker closer to the camera.
* **Wrong color detected**: Make sure you're clicking the correct color patches in the right order.
* **Calibration fails**: Try recapturing the reference image with better lighting.

---

## What This Calibration Does
The color calibration:
1. Captures a reference image of your ColorChecker chart
2. Extracts LAB color values from the 5 reference colors you select
3. Updates the system's color detection parameters
4. Improves accuracy of color recognition for red, green, and blue objects

After completing this calibration, the system will be better at detecting and tracking colored objects in the camera view. 