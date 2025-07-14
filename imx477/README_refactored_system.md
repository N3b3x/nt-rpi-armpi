# Advanced Tracking System - Refactored

This is a modular, refactored version of the gesture and face tracking system for the ArmPi Mini robot. The system has been broken down into reusable components that can be easily maintained, tested, and extended.

## Architecture

The system is composed of several modular components:

### Core Components

1. **GestureDetector** (`gesture_detector.py`)
   - Handles hand gesture recognition using MediaPipe Hands
   - Supports multiple gestures: Pointing, Grab, Open Hand, Fist, Peace, Thumb Up
   - Provides hand landmark detection and drawing

2. **FaceDetector** (`face_detector.py`)
   - Handles face detection and recognition using MediaPipe
   - Simple face recognition using MediaPipe FaceMesh features
   - Maintains a database of known faces

3. **MotionDetector** (`motion_detector.py`)
   - Detects motion in video frames
   - Controls spotlight based on motion detection
   - Provides motion timeout functionality

4. **DepthEstimator** (`depth_estimator.py`)
   - Estimates depth based on object area and arm position
   - Provides depth-based color coding for visualization
   - Maintains depth history for smoothing

5. **ZoomController** (`zoom_controller.py`)
   - Handles camera zoom functionality
   - Supports multiple zoom levels
   - Provides coordinate mapping for zoomed frames

6. **ArmController** (extended `arm_controller.py`)
   - Controls robotic arm movements
   - Added tracking functionality with PID control
   - Safe initialization for face scanning

7. **CameraProcessor** (extended `camera_processor.py`)
   - Handles camera operations and image processing
   - Added IMX477 support and tracking functionality
   - Provides frame processing for tracking applications

### Main System

8. **AdvancedTrackingSystem** (`advanced_tracking_system.py`)
   - Integrates all components into a complete system
   - Provides user interface and controls
   - Handles mode switching and system state

## Features

### Detection Modes
- **Face Detection**: Detects and recognizes faces using MediaPipe
- **Hand Gesture Detection**: Detects hand gestures using MediaPipe Hands

### Tracking Features
- **Motion Detection**: Automatically detects motion and controls spotlight
- **Depth Estimation**: Estimates object depth based on area and arm position
- **Zoom Control**: Multiple zoom levels with coordinate mapping
- **Arm Tracking**: PID-controlled arm tracking of detected objects

### User Controls
- `f` - Switch to Face Detection Mode
- `h` - Switch to Hand Gesture Detection Mode
- `z` - Cycle through zoom levels
- `l` - Toggle spotlight
- `u` - Toggle undistort
- `t` - Toggle tracking
- `q` - Quit

## Installation and Setup

### Prerequisites
- Python 3.7+
- OpenCV
- MediaPipe
- NumPy
- Picamera2
- ArmPi Mini SDK

### Dependencies
```bash
pip install opencv-python mediapipe numpy picamera2
```

### Camera Setup
The system is configured for the IMX477 camera with:
- Resolution: 1920x1080
- Frame rate: ~50 FPS
- Auto white balance and exposure

## Usage

### Testing Components
```bash
cd imx477
python3 -m tests.test_modular_system
```

### Testing Imports
```bash
cd imx477
python3 -m tests.test_imports
```

### Running the Full System
```bash
cd imx477
python3 gesture_face_tracking.py
```

### Individual Component Testing
Each component can be tested independently:

```python
from tracking_components.gesture_detector import GestureDetector
from tracking_components.face_detector import FaceDetector
from tracking_components.motion_detector import MotionDetector
from tracking_components.depth_estimator import DepthEstimator
from tracking_components.zoom_controller import ZoomController

# Initialize components
gesture_detector = GestureDetector()
face_detector = FaceDetector()
motion_detector = MotionDetector()
depth_estimator = DepthEstimator()
zoom_controller = ZoomController()
```

Or import the entire package:

```python
from tracking_components import GestureDetector, FaceDetector, MotionDetector, DepthEstimator, ZoomController

# Initialize components
gesture_detector = GestureDetector()
face_detector = FaceDetector()
motion_detector = MotionDetector()
depth_estimator = DepthEstimator()
zoom_controller = ZoomController()
```

## Component Details

### GestureDetector
- Uses MediaPipe Hands for hand landmark detection
- Analyzes finger positions to determine gestures
- Supports 6 different gestures with configurable thresholds

### FaceDetector
- Uses MediaPipe FaceDetection for face detection
- Uses MediaPipe FaceMesh for face recognition
- Maintains a simple feature-based recognition database
- Saves/loads known faces to/from file

### MotionDetector
- Uses frame differencing for motion detection
- Configurable thresholds for motion sensitivity
- Automatic spotlight control based on motion
- Motion timeout functionality

### DepthEstimator
- Estimates depth based on object area
- Uses arm position as additional depth reference
- Maintains depth history for smoothing
- Provides depth-based color coding

### ZoomController
- Supports multiple zoom levels (1.0x to 3.0x)
- Provides coordinate mapping for zoomed frames
- Handles edge cases and boundary conditions

### ArmController Extensions
- Added `track_target()` method for PID-controlled tracking
- Added `set_target_depth()` for depth-based tracking
- Added `get_tracking_status()` for monitoring
- Safe initialization with `init_move_face_scan()`

### CameraProcessor Extensions
- Added IMX477-specific settings
- Added tracking state management
- Added frame processing for tracking applications

## Configuration

### Camera Settings
- Resolution: 1920x1080 for detection, 640x480 for display
- Frame rate: ~50 FPS
- Auto controls enabled

### Detection Settings
- Face detection confidence: 0.8
- Hand detection confidence: 0.7
- Motion threshold: 25 pixels
- Motion area threshold: 500 pixels

### Tracking Settings
- PID parameters for smooth arm tracking
- Depth estimation with history smoothing
- Configurable zoom levels

## Troubleshooting

### Common Issues

1. **Camera not working**
   - Check IMX477 camera connection
   - Verify Picamera2 installation
   - Check camera permissions

2. **Arm not responding**
   - Check servo connections
   - Verify SDK installation
   - Check deviation data

3. **Detection not working**
   - Check MediaPipe installation
   - Verify camera calibration
   - Check lighting conditions

4. **Performance issues**
   - Reduce camera resolution
   - Lower detection confidence thresholds
   - Disable unused features

### Debug Mode
Enable debug output by setting environment variable:
```bash
export DEBUG=1
python3 advanced_tracking_system.py
```

## Extending the System

### Adding New Gestures
1. Modify `GestureDetector._analyze_gesture()`
2. Add gesture logic based on landmark positions
3. Update `GESTURES` dictionary

### Adding New Detection Types
1. Create new detector class following the same pattern
2. Implement detection and drawing methods
3. Integrate into `AdvancedTrackingSystem`

### Customizing Arm Behavior
1. Modify `ArmController.track_target()`
2. Adjust PID parameters
3. Add new movement patterns

## File Structure

```
imx477/
├── tracking_components/         # Reusable tracking components
│   ├── __init__.py            # Package initialization
│   ├── gesture_detector.py    # Hand gesture detection
│   ├── face_detector.py       # Face detection and recognition
│   ├── motion_detector.py     # Motion detection and spotlight control
│   ├── depth_estimator.py     # Depth estimation and visualization
│   └── zoom_controller.py     # Camera zoom functionality
├── tests/                     # Test files
│   ├── __init__.py           # Test package initialization
│   ├── test_camera.py        # Camera tests
│   ├── test_modular_system.py # Component testing
│   └── test_imports.py       # Import verification
├── gesture_face_tracking.py   # Main refactored system
├── arm_controller.py          # Extended arm controller
├── camera_processor.py        # Extended camera processor
└── README_refactored_system.md # This documentation
```

## Performance Considerations

- The system runs at ~50 FPS on the Raspberry Pi
- Face detection is more computationally intensive than hand detection
- Motion detection adds minimal overhead
- Zoom operations are performed on CPU (consider GPU acceleration for higher zoom levels)

## Future Improvements

1. **GPU Acceleration**: Use GPU for image processing operations
2. **Multi-threading**: Parallel processing of different detection types
3. **Machine Learning**: Replace simple feature matching with trained models
4. **3D Tracking**: Add stereo vision or depth camera support
5. **Gesture Training**: Allow users to train custom gestures
6. **Cloud Integration**: Upload detection data for analysis

## Contributing

When extending the system:
1. Follow the existing component patterns
2. Add proper documentation
3. Include unit tests
4. Update this README
5. Test with the existing test suite 