# Complete Camera Calibration Fix Guide

## 🎯 Problem Summary

Your camera is blurry because **automatic fisheye distortion correction is being applied** by the old `Camera.py` file. This uses interpolation that can make images appear blurry if:
- The calibration data is poor quality
- The calibration was done with different camera settings
- The distortion coefficients don't match your current camera setup

## 🔧 Quick Fix Solutions

### Option 1: Disable Automatic Calibration (Recommended)

```bash
# Run the calibration fix tool
python3 fix_camera_calibration.py

# Follow the prompts:
# 1. First backup your calibration files
# 2. Remove the problematic old calibration file
```

### Option 2: Test Camera Systems

```bash
# Run the camera test tool
python3 test_camera_calibration.py

# This will let you:
# - Toggle calibration on/off in real time
# - Compare old vs new camera systems
# - See the difference immediately
```

### Option 3: Manual Quick Fix

```bash
# Backup and remove the problematic file
mv /workspace/CameraCalibration/calibration_param.npz /workspace/CameraCalibration/calibration_param.npz.backup
```

## 📊 Understanding Your Camera System

Your workspace has **two separate camera implementations**:

### 1. Old System (`Camera.py`)
- **Location**: `/workspace/Camera.py`
- **Calibration**: `/workspace/CameraCalibration/calibration_param.npz`
- **Behavior**: **ALWAYS applies calibration** (this causes blur)
- **Problem**: No easy way to disable calibration

### 2. New System (`imx477/Camera.py`)
- **Location**: `/workspace/imx477/Camera.py`
- **Calibration**: `/workspace/imx477/calibration_data.npz`
- **Behavior**: Raw camera feed by default
- **Control**: Press `u` key to toggle calibration in IMX477 apps

## 🎛️ Camera Control Keys

When using IMX477 applications:
- **`u`** = Toggle undistortion on/off
- **`q`** = Quit application
- **`c`** = Capture/calibrate (in calibration modes)

Applications with `u` key support:
- `imx477/face_tracker.py`
- `imx477/face_detect.py`
- `imx477/gesture_face_tracking.py`
- `imx477/lab_adjust.py`

## 🔍 Diagnostic Commands

### Check Calibration Status
```bash
python3 fix_camera_calibration.py
# Choose option 3 to check status
```

### Test Camera Quality
```bash
python3 test_camera_calibration.py
# Choose option 1 to test with calibration toggle
```

### View Calibration Files
```bash
ls -la /workspace/CameraCalibration/calibration_param.npz
ls -la /workspace/imx477/calibration_data.npz
```

## 🎯 Proper Camera Calibration Guide

If you want to create **good calibration data** instead of just disabling it:

### Prerequisites
- High-quality printed checkerboard (6x9 or 7x7 internal corners)
- Good lighting
- Steady hands
- At least 15-20 calibration images

### Step-by-Step Calibration

#### Method 1: Using lab_adjust.py (Recommended)
```bash
python3 imx477/lab_adjust.py

# In the application:
# 1. Press 'd' for distortion calibration
# 2. Follow on-screen instructions
# 3. Move checkerboard to many different positions
# 4. Capture 15-20 images from different angles
# 5. Let it process the calibration
```

#### Method 2: Using the calibration script directly
```bash
python3 imx477/lab_auto_calibration.py
```

### Calibration Tips for Best Results

1. **Checkerboard Coverage**:
   - Top, bottom, left, right corners of image
   - Center of image
   - Various distances (close and far)
   - Different tilt angles

2. **Image Quality**:
   - Hold checkerboard very steady when capturing
   - Ensure good lighting on the checkerboard
   - Make sure all corners are visible
   - Avoid motion blur

3. **Capture Strategy**:
   ```
   Position 1: Center, straight on
   Position 2: Top-left corner, tilted
   Position 3: Top-right corner, tilted
   Position 4: Bottom-left corner, tilted
   Position 5: Bottom-right corner, tilted
   Position 6: Close to camera, center
   Position 7: Far from camera, center
   Position 8: Rotated 45 degrees
   ... continue for 15-20 total positions
   ```

### Validation After Calibration

```bash
# Test the new calibration
python3 imx477/lab_adjust.py
# Press 'u' to toggle calibration and see if it improves the image
```

## 🐛 Troubleshooting

### Problem: Still Blurry After Fix
**Solution**: Check if other applications are using the old Camera.py
```bash
grep -r "from Camera import" /workspace/
# Update any files found to use IMX477 camera instead
```

### Problem: Calibration Not Working
**Solutions**:
1. Use better lighting
2. Print a higher quality checkerboard
3. Capture more images from different angles
4. Ensure checkerboard is completely flat

### Problem: Can't Find Checkerboard Corners
**Solutions**:
1. Increase contrast/lighting
2. Use a larger checkerboard pattern
3. Ensure pattern is not reflective
4. Try different camera settings

### Problem: Camera Not Found
**Solutions**:
```bash
# Check camera connection
lsusb | grep -i camera
v4l2-ctl --list-devices

# Test basic camera access
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"
```

## ⚙️ Advanced Configuration

### Custom Camera Settings
Edit the camera settings in `/workspace/imx477/lab_auto_calibration.py`:

```python
lab_auto_calibration.set_controls(picam2,
    awb_enable=False,
    colour_gains=(1.5, 1.2),  # Red, Blue gains
    ae_enable=False,
    exposure_time=10000,      # microseconds
    analogue_gain=1.0,
    sharpness=8.0,           # Reduce if too sharp
    contrast=1.0,
    saturation=1.0,
    brightness=0.0
)
```

### Using Different Checkerboard Sizes
In calibration scripts, modify:
```python
checkerboard = (7, 7)  # Change to your pattern size
square_size = 20.0     # Change to your square size in mm
```

## 📝 File Reference

### Important Files
- `/workspace/Camera.py` - Old camera system (modified with calibration toggle)
- `/workspace/imx477/Camera.py` - New camera system (raw feed)
- `/workspace/test_camera_calibration.py` - Test tool you can run
- `/workspace/fix_camera_calibration.py` - Fix tool you can run

### Calibration Data Files
- `/workspace/CameraCalibration/calibration_param.npz` - Old system (causes blur)
- `/workspace/imx477/calibration_data.npz` - New system (optional)

### Backup Files (created by fix tool)
- `calibration_backup_YYYYMMDD_HHMMSS/` - Timestamped backups

## 🎉 Summary

1. **Immediate Fix**: Run `python3 fix_camera_calibration.py` and choose option 2
2. **Test Results**: Run `python3 test_camera_calibration.py` 
3. **Long-term**: Use IMX477 applications for better camera control
4. **If needed**: Recalibrate properly with checkerboard using `imx477/lab_adjust.py`

The blur should be completely gone after removing the problematic calibration file!