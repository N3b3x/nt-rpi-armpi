#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import numpy as np
from picamera2 import Picamera2

# Add SDK paths
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')
sys.path.append('/home/pi/ArmPi_mini/')

# Import our modular components
from ..tracking_components.gesture_detector import GestureDetector
from ..tracking_components.face_detector import FaceDetector
from ..tracking_components.motion_detector import MotionDetector
from ..tracking_components.depth_estimator import DepthEstimator
from ..tracking_components.zoom_controller import ZoomController

def test_components():
    """Test all modular components."""
    print("=== Testing Modular Components ===")
    
    # Test GestureDetector
    print("\n1. Testing GestureDetector...")
    gesture_detector = GestureDetector()
    print(f"   - GestureDetector initialized with {len(gesture_detector.GESTURES)} gestures")
    print(f"   - Available gestures: {list(gesture_detector.GESTURES.keys())}")
    
    # Test FaceDetector
    print("\n2. Testing FaceDetector...")
    face_detector = FaceDetector()
    print(f"   - FaceDetector initialized")
    print(f"   - Known faces: {len(face_detector.known_faces)}")
    
    # Test MotionDetector
    print("\n3. Testing MotionDetector...")
    motion_detector = MotionDetector()
    print(f"   - MotionDetector initialized")
    print(f"   - Motion threshold: {motion_detector.motion_threshold}")
    print(f"   - Motion area threshold: {motion_detector.motion_area_threshold}")
    
    # Test DepthEstimator
    print("\n4. Testing DepthEstimator...")
    depth_estimator = DepthEstimator()
    print(f"   - DepthEstimator initialized")
    print(f"   - Reference distance: {depth_estimator.reference_distance}cm")
    print(f"   - Reference size: {depth_estimator.reference_size}")
    
    # Test ZoomController
    print("\n5. Testing ZoomController...")
    zoom_controller = ZoomController()
    print(f"   - ZoomController initialized")
    print(f"   - Available zoom levels: {zoom_controller.get_zoom_levels()}")
    print(f"   - Current zoom: {zoom_controller.get_current_zoom()}x")
    
    # Test zoom cycling
    print("\n6. Testing zoom cycling...")
    for i in range(3):
        zoom_level = zoom_controller.cycle_zoom()
        print(f"   - Zoom level {i+1}: {zoom_level}x")
    
    # Reset zoom
    zoom_controller.reset()
    print(f"   - Reset zoom to: {zoom_controller.get_current_zoom()}x")
    
    print("\n=== All Components Tested Successfully ===")

def test_camera_integration():
    """Test camera integration with components."""
    print("\n=== Testing Camera Integration ===")
    
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (1920, 1080)},
        controls={"FrameDurationLimits": (19989, 19989)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    
    # Force auto controls
    picam2.set_controls({
        "AwbEnable": True,
        "AeEnable": True,
        "AwbMode": 0
    })
    
    # Initialize components
    gesture_detector = GestureDetector()
    face_detector = FaceDetector()
    motion_detector = MotionDetector()
    depth_estimator = DepthEstimator()
    zoom_controller = ZoomController()
    
    print("Camera initialized. Press 'q' to quit, 'f' for face mode, 'h' for hand mode")
    
    detection_mode = 'face'
    frame_count = 0
    
    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_count += 1
            
            # Apply zoom
            if zoom_controller.get_current_zoom() > 1.0:
                frame_bgr = zoom_controller.apply_zoom(frame_bgr)
            
            # Detect motion
            motion_center, motion_area = motion_detector.detect_motion(frame_bgr)
            if motion_center is not None:
                frame_bgr = motion_detector.draw_motion_detection(frame_bgr, motion_center, motion_area)
            
            # Detect based on mode
            if detection_mode == 'face':
                result = face_detector.detect_face(frame_bgr)
                if result[0] is not None:
                    center, area, name = result
                    frame_bgr = face_detector.draw_face_detection(frame_bgr, center, area, name)
                    
                    # Estimate depth
                    depth, confidence = depth_estimator.estimate_depth(area, 18)
                    frame_bgr = depth_estimator.draw_depth_info(frame_bgr, depth, confidence)
            else:
                result = gesture_detector.detect_hand(frame_bgr)
                if result[0] is not None:
                    center, area, gesture = result
                    if gesture:
                        cv2.putText(frame_bgr, f'Gesture: {gesture}', 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
                    
                    # Estimate depth
                    depth, confidence = depth_estimator.estimate_depth(area, 18)
                    frame_bgr = depth_estimator.draw_depth_info(frame_bgr, depth, confidence)
            
            # Draw zoom info
            frame_bgr = zoom_controller.draw_zoom_info(frame_bgr)
            
            # Draw mode info
            cv2.putText(frame_bgr, f'Mode: {detection_mode.upper()}', 
                       (10, frame_bgr.shape[0] - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(frame_bgr, f'Frame: {frame_count}', 
                       (10, frame_bgr.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            
            # Resize for display
            display_frame = cv2.resize(frame_bgr, (640, 480))
            cv2.imshow('Modular System Test', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                detection_mode = 'face'
                print("Switched to face detection mode")
            elif key == ord('h'):
                detection_mode = 'hand'
                print("Switched to hand detection mode")
            elif key == ord('z'):
                zoom_controller.cycle_zoom()
                print(f"Zoom level: {zoom_controller.get_current_zoom()}x")
        else:
            time.sleep(0.01)
    
    cv2.destroyAllWindows()
    print("Camera test completed")

if __name__ == '__main__':
    try:
        test_components()
        test_camera_integration()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc() 