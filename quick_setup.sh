#!/bin/bash

# ArmPi Mini Quick Setup Script for Raspberry Pi 5
# This script automates the initial setup process

set -e  # Exit on any error

echo "=========================================="
echo "ArmPi Mini Setup for Raspberry Pi 5"
echo "=========================================="

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Warning: This script is designed for Raspberry Pi. Continue anyway? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "Step 1: Updating system packages..."
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

# Install system dependencies
echo "Step 2: Installing system dependencies..."
sudo apt install -y python3-pip python3-dev python3-venv
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libatlas-base-dev liblapack-dev
sudo apt install -y libhdf5-dev libhdf5-serial-dev
sudo apt install -y libqtgui4 libqtwebkit4 libqt4-test python3-pyqt5
sudo apt install -y libjasper-dev libqtcore4 libqt4-test
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install -y libgtk-3-dev
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install -y libxvidcore-dev libx264-dev
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt install -y libatlas-base-dev gfortran
sudo apt install -y libtiff5-dev libjasper-dev libilmbase-dev libopenexr-dev libgstreamer1.0-dev
sudo apt install -y htop git curl wget

# Add user to necessary groups
echo "Step 3: Setting up user permissions..."
sudo usermod -a -G video $USER
sudo usermod -a -G dialout $USER

# Create virtual environment
echo "Step 4: Setting up Python environment..."
cd ~/nt_rpi_armpi
python3 -m venv armpi_env
source armpi_env/bin/activate

# Install Python packages
echo "Step 5: Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install numpy scipy opencv-python matplotlib PyYAML

# Install additional requirements if file exists
if [ -f "requirements_jacobian.txt" ]; then
    echo "Installing Jacobian requirements..."
    pip3 install -r requirements_jacobian.txt
fi

# Check hardware connections
echo "Step 6: Checking hardware connections..."
echo "USB devices:"
lsusb

echo "Serial ports:"
ls /dev/tty* 2>/dev/null || echo "No serial ports found"

echo "Camera devices:"
ls /dev/video* 2>/dev/null || echo "No camera devices found"

# Test basic imports
echo "Step 7: Testing Python imports..."
python3 -c "
import sys
sys.path.append('/home/pi/nt_rpi_armpi/')
try:
    import cv2
    print('✓ OpenCV imported successfully')
except ImportError as e:
    print('✗ OpenCV import failed:', e)

try:
    import numpy as np
    print('✓ NumPy imported successfully')
except ImportError as e:
    print('✗ NumPy import failed:', e)

try:
    from kinematics.arm_move_ik import ArmIK
    print('✓ ArmPi kinematics imported successfully')
except ImportError as e:
    print('✗ ArmPi kinematics import failed:', e)
"

# Create activation script
echo "Step 8: Creating activation script..."
cat > activate_armpi.sh << 'EOF'
#!/bin/bash
cd ~/nt_rpi_armpi
source armpi_env/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/pi/nt_rpi_armpi
echo "ArmPi environment activated!"
echo "Run: python3 ArmPi_mini.py"
EOF

chmod +x activate_armpi.sh

# Create test script
echo "Step 9: Creating test script..."
cat > test_armpi.sh << 'EOF'
#!/bin/bash
cd ~/nt_rpi_armpi
source armpi_env/bin/activate

echo "Testing ArmPi components..."

# Test camera
echo "Testing camera..."
python3 Camera.py &
CAMERA_PID=$!
sleep 3
kill $CAMERA_PID 2>/dev/null || true

# Test kinematics
echo "Testing kinematics..."
python3 -c "
import sys
sys.path.append('/home/pi/nt_rpi_armpi/')
from kinematics.arm_move_ik import ArmIK
ak = ArmIK()
print('✓ Kinematics initialized successfully')
"

echo "Basic tests completed!"
EOF

chmod +x test_armpi.sh

echo "=========================================="
echo "Setup completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. Activate the environment: source activate_armpi.sh"
echo "3. Test the system: ./test_armpi.sh"
echo "4. Start the main program: python3 ArmPi_mini.py"
echo ""
echo "For detailed instructions, see SETUP_GUIDE.md"
echo ""
echo "Hardware checklist:"
echo "□ ArmPi Mini board connected via USB"
echo "□ Servo motors connected to board"
echo "□ Camera connected"
echo "□ Power supply connected to ArmPi board"
echo ""
echo "After reboot, you can access:"
echo "- Camera feed: http://$(hostname -I | awk '{print $1}'):8080"
echo "- RPC server: localhost:5000" 