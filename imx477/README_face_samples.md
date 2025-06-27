# IMX477 Face Detection and Recognition Samples

This directory contains migrated face detection and recognition samples specifically adapted for the IMX477 camera, replacing the original USB camera versions.

## Files

### `face_detect.py`
- **Purpose**: Basic face detection using MediaPipe
- **Features**: 
  - Detects faces in real-time using the IMX477 camera
  - Draws bounding boxes around detected faces
  - Triggers buzzer when face is detected
  - Uses ArmController for hardware access
  - No arm movement (simple detection only)

### `face_recognition.py`
- **Purpose**: Face detection with robotic arm interaction
- **Features**:
  - Detects faces in real-time using the IMX477 camera
  - Draws bounding boxes around detected faces
  - Controls gripper servo when face is detected
  - **Enhanced polar scanning pattern** covering ±180° range
  - Multiple scan positions at each angle for better coverage
  - Full robotic arm integration using ArmController
  - Proper servo movement with angle tracking

## Key Changes from USB Camera Version

1. **Camera Integration**:
   - Replaced `cv2.VideoCapture()` with IMX477 `Camera` class
   - Updated resolution to 640x480 (IMX477 native)
   - Removed USB camera stream URL dependencies

2. **Arm Control Integration**:
   - Replaced direct board servo control with `ArmController` class
   - Uses proper servo movement methods (`open_gripper()`, `close_gripper()`, `move_base()`)
   - Better angle tracking and limits
   - Consistent with rest of codebase

3. **Import Updates**:
   - Added `from Camera import Camera`
   - Added `from color_palletizing.arm_controller import ArmController`
   - Removed direct board and kinematics imports

4. **Resolution Updates**:
   - Changed from 320x240 to 640x480
   - Updated display window sizes

5. **Code Cleanup**:
   - Removed unused imports and variables
   - Updated comments and documentation
   - Improved error handling
   - Better separation of concerns

## Usage

### Running Face Detection
```bash
cd /home/pi/ArmPi_mini/imx477
python3 face_detect.py
```

### Running Face Recognition with Arm
```bash
cd /home/pi/ArmPi_mini/imx477
python3 face_recognition.py
```

## Requirements

- IMX477 camera properly configured
- MediaPipe library installed: `pip3 install mediapipe`
- ArmPi Mini SDK and dependencies
- Proper camera calibration (optional but recommended)

## Features

### Face Detection (`face_detect.py`)
- Real-time face detection using MediaPipe
- Visual feedback with green bounding boxes
- Audio feedback (buzzer) when face detected
- Uses ArmController for hardware access
- ESC key to exit

### Face Recognition (`face_recognition.py`)
- All features from face detection
- Gripper servo control (opens/closes when face detected)
- Base servo scanning movement with proper angle limits
- Robotic arm integration using ArmController
- ESC key to exit

## Technical Improvements

### Enhanced Scanning Pattern
- **Before**: Simple left-right movement with hard-coded limits
- **After**: **Polar scanning pattern** covering ±180° range with 60° intervals
- **Benefits**: Much better coverage area, more efficient face detection

### Arm Control
- **Before**: Direct `board.pwm_servo_set_position()` calls
- **After**: Clean `arm_controller.open_gripper()` and `arm_controller.move_base()` methods
- **Benefits**: Better error handling, angle tracking, and consistency

### Movement Limits
- **Before**: Hard-coded PWM limits (1100-1900)
- **After**: Angle-based limits with intelligent scanning pattern
- **Benefits**: More intuitive control and safer operation

### Code Organization
- **Before**: Mixed hardware control and vision processing
- **After**: Clean separation using ArmController abstraction
- **Benefits**: Easier maintenance and debugging

### Scanning Strategy
- **Coverage**: ±180° range with 60° intervals (7 scan angles)
- **Positions**: Multiple radial positions at each angle using `get_scan_positions()`
- **Detection**: 3 detection attempts at each position for reliability
- **Response**: Immediate greeting action when face detected

## Troubleshooting

1. **Camera not working**: Ensure IMX477 is properly connected and configured
2. **MediaPipe errors**: Install with `pip3 install mediapipe`
3. **Arm not moving**: Check servo connections and power
4. **Poor detection**: Ensure good lighting and camera positioning
5. **Import errors**: Ensure all dependencies are installed and paths are correct

## Performance Notes

- IMX477 provides higher resolution (640x480) than original USB camera
- MediaPipe face detection works well with the higher resolution
- Processing may be slightly slower due to increased resolution
- Consider adjusting `min_detection_confidence` if needed
- ArmController provides smoother servo movements

## Customization

You can modify these parameters in both files:
- `min_detection_confidence=0.8`: Detection sensitivity
- `scores[0] > 0.7`: Confidence threshold for actions
- Base movement limits: `arm_controller.current_base_angle >= 45 or <= -45`
- Camera resolution (in Camera initialization)
- Movement timing and delays 