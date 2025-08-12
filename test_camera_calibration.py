#!/usr/bin/env python3
"""
Test script to compare camera feed with and without calibration correction.
This will help you see the difference and choose the best option.
"""
import cv2
import time
import sys
sys.path.append('/home/pi/ArmPi_mini/')

# Import both camera systems
from Camera import Camera as OldCamera
from imx477.Camera import Camera as IMX477Camera

def test_old_camera_system():
    """Test the old camera system with calibration toggle"""
    print("\n=== Testing Old Camera System ===")
    print("Starting with calibration DISABLED (should be clearer)")
    
    # Test without calibration (should be clearer)
    camera_no_calib = OldCamera(enable_calibration=False)
    camera_no_calib.camera_open()
    
    print("Press 'c' to toggle calibration, 'q' to quit")
    
    calibration_enabled = False
    while True:
        frame = camera_no_calib.frame
        if frame is not None:
            # Add status text
            status_text = "Calibration: ON (May be blurry)" if calibration_enabled else "Calibration: OFF (Raw feed)"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'c' to toggle, 'q' to quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('Old Camera System Test', frame)
            
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            calibration_enabled = not calibration_enabled
            camera_no_calib.set_calibration(calibration_enabled)
            print(f"Calibration {'enabled' if calibration_enabled else 'disabled'}")
        elif key == ord('q'):
            break
            
    camera_no_calib.camera_close()
    cv2.destroyAllWindows()

def test_imx477_camera_system():
    """Test the IMX477 camera system (newer, better implementation)"""
    print("\n=== Testing IMX477 Camera System ===")
    print("This system has optional calibration that you can toggle with 'u' key")
    
    try:
        camera = IMX477Camera()
        print("Press 'q' to quit")
        
        while True:
            frame = camera.get_frame()
            if frame is not None:
                # Convert RGB to BGR for OpenCV display
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.putText(frame_bgr, "IMX477 Camera (Raw feed)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame_bgr, "Press 'q' to quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('IMX477 Camera Test', frame_bgr)
                
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                
    except Exception as e:
        print(f"IMX477 camera not available or error: {e}")
        print("This is normal if you're not using the IMX477 camera module")
        
    cv2.destroyAllWindows()

def main():
    print("Camera Calibration Test Tool")
    print("============================")
    print("This tool will help you test different camera settings")
    print("to determine what's causing the blur in your camera feed.")
    print()
    
    while True:
        print("\nChoose test option:")
        print("1. Test Old Camera System (with calibration toggle)")
        print("2. Test IMX477 Camera System")
        print("3. Quick comparison (both systems)")
        print("q. Quit")
        
        choice = input("Enter your choice (1/2/3/q): ").strip().lower()
        
        if choice == '1':
            test_old_camera_system()
        elif choice == '2':
            test_imx477_camera_system()
        elif choice == '3':
            print("\nTesting both systems quickly...")
            test_old_camera_system()
            time.sleep(1)
            test_imx477_camera_system()
        elif choice == 'q':
            break
        else:
            print("Invalid choice, please try again.")
    
    print("Test completed!")

if __name__ == '__main__':
    main()