#!/usr/bin/python3
# coding=utf8
"""
Test script to verify that all imports work correctly with the new folder structure.
"""

import sys
import os

# Add SDK paths
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/common_sdk')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics')
sys.path.insert(0, '/home/pi/ArmPi_mini/armpi_mini_sdk/kinematics_sdk')
sys.path.append('/home/pi/ArmPi_mini/')

def test_imports():
    """Test all imports to ensure they work correctly."""
    print("=== Testing Imports ===")
    
    try:
        # Test tracking components imports
        print("1. Testing tracking_components imports...")
        from ..tracking_components.gesture_detector import GestureDetector
        from ..tracking_components.face_detector import FaceDetector
        from ..tracking_components.motion_detector import MotionDetector
        from ..tracking_components.depth_estimator import DepthEstimator
        from ..tracking_components.zoom_controller import ZoomController
        print("   ✓ All tracking_components imports successful")
        
        # Test package import
        print("2. Testing package import...")
        from ..tracking_components import GestureDetector as GD, FaceDetector as FD
        print("   ✓ Package import successful")
        
        # Test existing components
        print("3. Testing existing component imports...")
        from ..arm_controller import ArmController
        from ..camera_processor import CameraProcessor
        print("   ✓ Existing component imports successful")
        
        # Test component initialization
        print("4. Testing component initialization...")
        gesture_detector = GestureDetector()
        face_detector = FaceDetector()
        motion_detector = MotionDetector()
        depth_estimator = DepthEstimator()
        zoom_controller = ZoomController()
        print("   ✓ All components initialized successfully")
        
        print("\n=== All Import Tests Passed ===")
        return True
        
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

if __name__ == '__main__':
    success = test_imports()
    if success:
        print("\n✅ All imports working correctly!")
    else:
        print("\n❌ Some imports failed!")
        sys.exit(1) 