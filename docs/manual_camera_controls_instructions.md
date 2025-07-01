# ArmPi Mini Manual Camera Controls Guide

## Objective
To manually adjust camera settings including exposure time, gain, and brightness for optimal image quality and color detection performance.

## Required Items
* IMX477 camera properly connected
* Good lighting conditions
* Objects or scenes to test camera settings

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Access Manual Camera Controls (Key: `m`)
1. Press the `m` key to enter Manual Camera Controls mode.
2. A new window titled "Camera Controls" will appear with live camera feed and control instructions.

---

## Step 3: Understanding the Controls
The manual camera controls provide real-time adjustment of camera parameters:

### Camera Parameter Controls
* **[e/r]** - **Exposure Time**: Adjust the exposure time (shutter speed)
  - `e`: Decrease exposure time (faster shutter, darker image)
  - `r`: Increase exposure time (slower shutter, brighter image)
  - Range: 1000-30000 microseconds

* **[g/h]** - **Gain**: Adjust the analog gain (amplification)
  - `g`: Decrease gain (less amplification, less noise)
  - `h`: Increase gain (more amplification, more noise)
  - Range: 1.0-10.0

* **[b/n]** - **Brightness**: Adjust the digital brightness
  - `b`: Decrease brightness (darker image)
  - `n`: Increase brightness (brighter image)
  - Range: -1.0 to 1.0

### Utility Controls
* **[m]** - **Apply**: Apply the current settings to the camera
* **[m]/[q]** - **Quit**: Exit Manual Camera Controls and return to main menu

---

## Step 4: Using Manual Camera Controls
1. **Start with current settings**: The camera will start with default or previously applied settings.
2. **Real-time feedback**: Current values for exposure, gain, and brightness are displayed on screen.
3. **Adjust incrementally**: Make small adjustments and observe the effect on the image.
4. **Apply changes**: Press `m` to apply your settings to the camera.
5. **Test with targets**: Use colored objects to test how settings affect color detection.

---

## Step 5: Optimizing for Color Detection
For best color detection performance:

### Exposure Settings
* **Too dark**: Increase exposure time (`r`) or gain (`h`)
* **Too bright**: Decrease exposure time (`e`) or gain (`g`)
* **Motion blur**: Use shorter exposure times
* **Low light**: Use longer exposure times with moderate gain

### Gain Settings
* **Low light**: Increase gain (`h`) but be aware of increased noise
* **Good lighting**: Use lower gain (`g`) for cleaner images
* **Noise reduction**: Lower gain reduces image noise

### Brightness Settings
* **Dark objects**: Increase brightness (`n`)
* **Bright objects**: Decrease brightness (`b`)
* **Balanced lighting**: Keep brightness near 0.0

---

## Step 6: Best Practices
* **Start conservative**: Begin with moderate settings and adjust gradually.
* **Test with targets**: Use the same colored objects you'll be detecting.
* **Consider lighting**: Adjust settings based on your lighting conditions.
* **Balance parameters**: Don't rely on just one parameter - balance exposure, gain, and brightness.
* **Apply frequently**: Press `m` regularly to apply your changes.

---

## Step 7: Exiting Manual Camera Controls
1. Press `m` or `q` to exit Manual Camera Controls.
2. Your last applied settings will remain active.
3. You'll return to the main menu.

---

## Important Notes
* **Real-time display**: Current parameter values are shown on screen.
* **Incremental changes**: Each key press changes values by small increments.
* **Applied vs. displayed**: Settings are only applied when you press `m`.
* **Persistence**: Applied settings remain active after exiting the controls.
* **Resolution**: Manual controls work with the current camera resolution.

---

## Troubleshooting
* **Image too dark**: Increase exposure time (`r`) or gain (`h`).
* **Image too bright**: Decrease exposure time (`e`) or gain (`g`).
* **Poor color detection**: Adjust brightness (`b`/`n`) and test with target objects.
* **Noisy image**: Reduce gain (`g`) and increase exposure time (`r`).
* **Motion blur**: Decrease exposure time (`e`) and increase gain (`h`).

---

## What Manual Camera Controls Do
Manual camera controls provide:
1. **Exposure control**: Adjust shutter speed for proper image brightness
2. **Gain control**: Adjust amplification for low-light situations
3. **Brightness control**: Fine-tune digital brightness levels
4. **Real-time feedback**: See current parameter values
5. **Immediate application**: Apply settings instantly to the camera

These controls are essential for:
* Optimizing image quality for color detection
* Adapting to different lighting conditions
* Troubleshooting camera performance issues
* Fine-tuning the system for specific environments
* Improving the accuracy of color recognition

---

## Parameter Ranges and Effects
* **Exposure Time**: 1000-30000 μs
  - Lower values: Faster shutter, darker image, less motion blur
  - Higher values: Slower shutter, brighter image, more motion blur

* **Gain**: 1.0-10.0
  - Lower values: Less amplification, cleaner image, less noise
  - Higher values: More amplification, brighter image, more noise

* **Brightness**: -1.0 to 1.0
  - Lower values: Darker image, better for bright objects
  - Higher values: Brighter image, better for dark objects

---

## Integration with Other Features
Manual camera controls work with:
* **Color detection**: Optimize settings for better color recognition
* **Undistortion**: Apply to both distorted and undistorted images
* **Calibration**: Use optimal settings during calibration procedures
* **Jog mode**: Settings persist during arm control operations

The settings you configure here will affect all camera operations in the system, so choose values that work well for your specific use case and lighting conditions. 