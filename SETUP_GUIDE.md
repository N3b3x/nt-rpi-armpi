# ArmPi Mini Setup Guide for Raspberry Pi 5

This guide will help you set up and run the ArmPi Mini robot on your Raspberry Pi 5.

## Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS (preferably the latest version)
- ArmPi Mini robot hardware
- USB camera (if not using the built-in camera)
- Internet connection for package installation

## Step 1: System Setup

### 1.1 Update Raspberry Pi OS
```bash
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
```

### 1.2 Install Required System Packages
```bash
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
```

### 1.3 Enable Camera and I2C (if needed)
```bash
sudo raspi-config
```
- Navigate to "Interface Options"
- Enable "Camera"
- Enable "I2C" (if using I2C sensors)
- Enable "Serial" (for UART communication)

### 1.4 Reboot
```bash
sudo reboot
```

## Step 2: Python Environment Setup

### 2.1 Create Virtual Environment (Optional but Recommended)
```bash
cd ~/nt_rpi_armpi
python3 -m venv armpi_env
source armpi_env/bin/activate
```

### 2.2 Install Python Dependencies
```bash
# Install basic requirements
pip3 install numpy scipy opencv-python matplotlib PyYAML

# Install additional requirements for advanced features
pip3 install -r requirements_jacobian.txt
```

## Step 3: Hardware Connection

### 3.1 Connect ArmPi Mini Hardware
1. Connect the ArmPi Mini board to the Raspberry Pi via USB
2. Connect the servo motors to the board
3. Connect the camera (if using external camera)
4. Connect power supply to the ArmPi Mini board

### 3.2 Check Hardware Connections
```bash
# Check if the board is detected
lsusb

# Check serial ports
ls /dev/tty*

# Check camera
ls /dev/video*
```

## Step 4: Configuration

### 4.1 Camera Calibration (if needed)
If you need to calibrate the camera for better accuracy:

```bash
cd CameraCalibration
python3 Calibration.py
```

Follow the on-screen instructions to calibrate your camera.

### 4.2 Check Configuration Files
Make sure these files exist and are properly configured:

```bash
# Check YAML configuration files
ls yaml/
cat yaml/deviation.yaml
cat yaml/lab_config.yaml
```

### 4.3 Update Path in Main Script
Edit `ArmPi_mini.py` to ensure the correct path:

```python
# Line 7 should point to your repository location
sys.path.append('/home/pi/nt_rpi_armpi/')
```

## Step 5: Testing Basic Functionality

### 5.1 Test Camera
```bash
python3 Camera.py
```
This should open the camera feed. Press 'q' to quit.

### 5.2 Test Hardware Interface
```bash
cd armpi_mini_demo
python3 hardware_test.py
```
This will test individual servo movements.

### 5.3 Test Basic Movement
```bash
python3 -c "
import sys
sys.path.append('/home/pi/nt_rpi_armpi/')
from kinematics.arm_move_ik import ArmIK
ak = ArmIK()
print('ArmPi initialized successfully')
"
```

## Step 6: Running the Main System

### 6.1 Start the Main Program
```bash
cd ~/nt_rpi_armpi
python3 ArmPi_mini.py
```

This will start:
- RPC server (for remote control)
- MJPEG streaming server (camera feed on port 8080)
- Main control loop

### 6.2 Access Camera Feed
Open a web browser and go to:
```
http://your_raspberry_pi_ip:8080
```

### 6.3 Test Remote Control
The RPC server runs on port 5000. You can test it with:

```bash
# Test RPC connection
python3 -c "
import jsonrpc
server = jsonrpc.ServerProxy('http://localhost:5000')
print(server.getLoadedFunc())
"
```

## Step 7: Running Different Functions

The system supports multiple functions. You can switch between them:

### 7.1 Available Functions
- **Function 1**: Remote Control (motion control)
- **Function 2**: Color Detection
- **Function 3**: Color Sorting
- **Function 4**: Color Tracking
- **Function 5**: Color Palletizing
- **Function 9**: LAB Calibration

### 7.2 Switch Functions via RPC
```python
import jsonrpc
server = jsonrpc.ServerProxy('http://localhost:5000')

# Load a function
server.loadFunc(2)  # Load color detection

# Start the function
server.startFunc()

# Stop the function
server.stopFunc()

# Unload the function
server.unloadFunc()
```

### 7.3 Test Color Detection
```bash
# Load color detection function
python3 -c "
import jsonrpc
server = jsonrpc.ServerProxy('http://localhost:5000')
server.loadFunc(2)
server.startFunc()
print('Color detection started')
"
```

## Step 8: Advanced Features

### 8.1 Jacobian-based Control
For smoother tracking and path planning:

```bash
# Install Jacobian requirements
pip3 install -r requirements_jacobian.txt

# Run Jacobian demo
python3 jacobian_path_planning_demo.py
```

### 8.2 Advanced Color Tracking
```bash
# Run advanced color tracking with Kalman filtering
python3 functions/advanced_color_tracking.py
```

## Step 9: Troubleshooting

### 9.1 Common Issues

**Camera not working:**
```bash
# Check camera permissions
sudo usermod -a -G video $USER
# Reboot and try again
```

**Servos not responding:**
```bash
# Check serial port permissions
sudo usermod -a -G dialout $USER
# Reboot and try again
```

**Import errors:**
```bash
# Make sure you're in the correct directory
cd ~/nt_rpi_armpi
# Check Python path
python3 -c "import sys; print(sys.path)"
```

### 9.2 Debug Mode
Run with debug logging:
```bash
python3 -u ArmPi_mini.py 2>&1 | tee debug.log
```

### 9.3 Check System Resources
```bash
# Monitor CPU and memory
htop

# Check disk space
df -h

# Check temperature
vcgencmd measure_temp
```

## Step 10: Auto-start (Optional)

To make the system start automatically on boot:

### 10.1 Create Systemd Service
```bash
sudo nano /etc/systemd/system/armpi-mini.service
```

Add the following content:
```ini
[Unit]
Description=ArmPi Mini Robot
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/nt_rpi_armpi
ExecStart=/usr/bin/python3 /home/pi/nt_rpi_armpi/ArmPi_mini.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### 10.2 Enable and Start Service
```bash
sudo systemctl daemon-reload
sudo systemctl enable armpi-mini
sudo systemctl start armpi-mini
sudo systemctl status armpi-mini
```

## Next Steps

1. **Calibrate the robot** using the LAB calibration function
2. **Test different functions** to understand capabilities
3. **Explore the Jacobian-based control** for advanced motion planning
4. **Set up ROS integration** if you want to use MoveIt (requires separate planning machine)
5. **Develop custom functions** for your specific use case

## Support

If you encounter issues:
1. Check the debug log: `tail -f debug.log`
2. Verify hardware connections
3. Check system resources
4. Review the configuration files
5. Test individual components

The system should now be fully operational on your Raspberry Pi 5! 