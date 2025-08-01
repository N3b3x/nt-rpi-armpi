#!/usr/bin/env python3
"""
Test script to verify ArmPi Mini Robot setup on ARM-based single-board computers
"""

import sys
import platform

def test_basic_imports():
    """Test basic Python imports"""
    print("Testing basic imports...")
    
    try:
        import numpy as np
        print(f"✓ NumPy {np.__version__}")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False
    
    try:
        import cv2
        print(f"✓ OpenCV {cv2.__version__}")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
        return False
    
    try:
        import yaml
        print("✓ PyYAML")
    except ImportError as e:
        print(f"✗ PyYAML import failed: {e}")
        return False
    
    try:
        import serial
        print("✓ PySerial")
    except ImportError as e:
        print(f"✗ PySerial import failed: {e}")
        return False
    
    try:
        import matplotlib
        print(f"✓ Matplotlib {matplotlib.__version__}")
    except ImportError as e:
        print(f"✗ Matplotlib import failed: {e}")
        return False
    
    return True

def detect_board_type():
    """Detect the type of ARM board"""
    system = platform.system()
    machine = platform.machine()
    
    if system == "Linux" and ("arm" in machine.lower() or "aarch64" in machine.lower()):
        try:
            with open('/proc/device-tree/model', 'r') as f:
                board_model = f.read().strip()
                if "Raspberry Pi" in board_model:
                    return "raspberry_pi", board_model
                elif "Jetson" in board_model:
                    return "jetson", board_model
                elif "Orange Pi" in board_model:
                    return "orange_pi", board_model
                elif "Rock Pi" in board_model:
                    return "rock_pi", board_model
                else:
                    return "generic_arm", board_model
        except FileNotFoundError:
            return "generic_arm", "Unknown ARM board"
    else:
        return "unsupported", f"Unsupported system: {system} {machine}"

def test_board_specific_imports():
    """Test board-specific imports"""
    board_type, board_model = detect_board_type()
    print(f"\nTesting board-specific imports...")
    print(f"Detected: {board_type} - {board_model}")
    
    if board_type == "raspberry_pi":
        try:
            import RPi.GPIO as GPIO
            print("✓ RPi.GPIO")
        except ImportError as e:
            print(f"✗ RPi.GPIO import failed: {e}")
        
        try:
            from picamera2 import Picamera2
            print("✓ Picamera2")
        except ImportError as e:
            print(f"✗ Picamera2 import failed: {e}")
    
    elif board_type == "jetson":
        try:
            import jtop
            print("✓ Jetson stats (jtop)")
        except ImportError as e:
            print(f"✗ Jetson stats import failed: {e}")
        
        try:
            from picamera2 import Picamera2
            print("✓ Picamera2")
        except ImportError as e:
            print(f"✗ Picamera2 import failed: {e}")
    
    elif board_type in ["orange_pi", "rock_pi", "generic_arm"]:
        try:
            from picamera2 import Picamera2
            print("✓ Picamera2")
        except ImportError as e:
            print(f"✗ Picamera2 import failed: {e}")
            print("  (This is normal for some ARM boards)")
    
    else:
        print("✗ Unsupported board type for ArmPi Mini Robot")

def test_project_imports():
    """Test project-specific imports"""
    print("\nTesting project imports...")
    
    try:
        # Add the SDK path
        import sys
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sdk_path = os.path.join(current_dir, 'armpi_mini_sdk', 'common_sdk')
        sys.path.append(sdk_path)
        
        import common.yaml_handle as yaml_handle
        print("✓ YAML handle module")
        
        # Test loading a config file
        data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)
        print(f"✓ Config file loaded: {data}")
        
    except Exception as e:
        print(f"✗ Project imports failed: {e}")

def test_hardware_interfaces():
    """Test hardware interface availability"""
    print("\nTesting hardware interfaces...")
    
    import os
    
    # Test serial ports
    serial_ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyS0']
    for port in serial_ports:
        if os.path.exists(port):
            print(f"✓ Serial port available: {port}")
        else:
            print(f"✗ Serial port not found: {port}")
    
    # Test camera interface
    if os.path.exists('/dev/video0'):
        print("✓ Camera interface available: /dev/video0")
    else:
        print("✗ Camera interface not found: /dev/video0")
    
    # Test GPIO (Raspberry Pi)
    if os.path.exists('/sys/class/gpio'):
        print("✓ GPIO interface available")
    else:
        print("✗ GPIO interface not available")

def main():
    """Main test function"""
    print("ArmPi Mini Robot Setup Test")
    print("=" * 50)
    print(f"Python version: {sys.version}")
    print(f"Platform: {platform.system()} {platform.machine()}")
    
    board_type, board_model = detect_board_type()
    print(f"Board: {board_type}")
    print(f"Model: {board_model}")
    print()
    
    # Check if this is a supported system
    if board_type == "unsupported":
        print("✗ This system is not supported for ArmPi Mini Robot")
        print("Please run this on an ARM-based single-board computer:")
        print("- Raspberry Pi")
        print("- NVIDIA Jetson")
        print("- Orange Pi")
        print("- Rock Pi")
        print("- Other ARM boards")
        return
    
    # Test basic imports
    basic_ok = test_basic_imports()
    
    # Test board-specific imports
    test_board_specific_imports()
    
    # Test project imports
    test_project_imports()
    
    # Test hardware interfaces
    test_hardware_interfaces()
    
    print("\n" + "=" * 50)
    if basic_ok:
        print("✓ Basic setup is working!")
        print("You can now run: python3 ArmPi_mini.py")
        
        if board_type == "raspberry_pi":
            print("\nRaspberry Pi specific notes:")
            print("- Make sure camera is enabled: sudo raspi-config")
            print("- Check serial permissions: ls -l /dev/ttyAMA0")
            print("- Monitor temperature: vcgencmd measure_temp")
        
        elif board_type == "jetson":
            print("\nJetson specific notes:")
            print("- Monitor performance: jtop")
            print("- Check GPU usage: nvidia-smi")
            print("- Monitor power: sudo tegrastats")
        
        else:
            print(f"\n{board_type.replace('_', ' ').title()} specific notes:")
            print("- Check serial port permissions")
            print("- Verify camera interface")
            print("- Test servo connections")
    else:
        print("✗ Some basic dependencies are missing.")
        print("Please run: ./auto-setup.sh")
    
    print("\nFor more information, see SETUP.md")

if __name__ == "__main__":
    main() 