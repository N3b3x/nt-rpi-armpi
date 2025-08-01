# ArmPi Mini Robot Setup Guide

This guide will help you set up the ArmPi Mini Robot project on ARM-based single-board computers. The ArmPi Mini is a robotic arm kit from HiWonder designed to work with Raspberry Pi and similar ARM boards.

## Supported Hardware

This setup is designed for ARM-based single-board computers:
- **Raspberry Pi** (all models: Pi 3, Pi 4, Pi Zero W, etc.)
- **NVIDIA Jetson** (Nano, AGX Xavier, Xavier NX)
- **Orange Pi** (all models)
- **Rock Pi** (all models)
- **Other ARM-based boards** (aarch64 or armv7l architecture)

## Quick Setup Options

### Option 1: Quick Start (Recommended for Beginners)
```bash
# Make scripts executable
chmod +x auto-setup.sh quick-start.sh

# Run the quick start script (runs setup + offers activation)
./quick-start.sh
```

### Option 2: Manual Setup (Recommended for Advanced Users)
```bash
# Make the setup script executable
chmod +x auto-setup.sh

# Run the automatic setup
./auto-setup.sh

# Activate the environment
source .venv/bin/activate

# Run the main application
python3 ArmPi_mini.py
```

### Option 3: Auto-Activation Setup (Optional)
If you want the virtual environment to automatically activate when you enter the project directory:

```bash
# First run the normal setup
./auto-setup.sh

# Then set up auto-activation
chmod +x setup-auto-activation.sh
./setup-auto-activation.sh

# Restart your terminal or reload shell config
source ~/.bashrc  # or ~/.zshrc for Zsh
```

## Manual Setup

If the automatic setup doesn't work for your board, you can follow the manual setup process below.

### Prerequisites

#### For Raspberry Pi:
- Raspberry Pi OS (Raspbian) or Ubuntu Server
- Python 3.8 or higher
- Camera module (for vision features)
- Serial connection to the ArmPi Mini robot board

#### For Jetson Nano:
- NVIDIA JetPack 4.6+ or JetPack 5.0+
- Python 3.8 or higher
- Camera module (for vision features)
- Serial connection to the ArmPi Mini robot board

#### For Other ARM Boards:
- Ubuntu 20.04+ or Debian-based OS
- Python 3.8 or higher
- Camera module (for vision features)
- Serial connection to the ArmPi Mini robot board

### Step 1: Install System Dependencies

#### Raspberry Pi:
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    libgtk2.0-dev \
    pkg-config \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5test5 \
    libcap-dev \
    libzmq3-dev \
    libcamera-apps \
    libcamera-tools \
    libcamera0 \
    libcamera-dev
```

#### Jetson Nano:
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    libgtk2.0-dev \
    pkg-config \
    libcap-dev \
    libzmq3-dev \
    libcamera-apps \
    libcamera-tools \
    libcamera0 \
    libcamera-dev
```

#### Other ARM Boards:
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    libgtk2.0-dev \
    pkg-config \
    libcap-dev \
    libzmq3-dev \
    libcamera-apps \
    libcamera-tools \
    libcamera0 \
    libcamera-dev
```

### Step 2: Create Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv .venv

# Activate virtual environment
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### Step 3: Install Python Dependencies

```bash
# Install all dependencies
pip install -r requirements.txt

# For Raspberry Pi, also install picamera2
pip install picamera2

# For Jetson, install jetson-stats
pip install jetson-stats
```

### Step 4: Setup Serial Permissions

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Set permissions for serial ports
sudo chmod 666 /dev/ttyAMA0
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyS0

# Reboot for changes to take effect
sudo reboot
```

### Step 5: Enable Interfaces (Raspberry Pi Only)

```bash
# Enable camera interface
sudo raspi-config nonint do_camera 0

# Enable I2C interface
sudo raspi-config nonint do_i2c 0

# Enable SPI interface
sudo raspi-config nonint do_spi 0

# Reboot for changes to take effect
sudo reboot
```

### Step 6: Verify Installation

```bash
# Activate the environment
source .venv/bin/activate

# Test imports
python3 -c "
import numpy as np
import cv2
import yaml
import serial
import matplotlib
print('All core dependencies imported successfully!')

# Test board-specific imports
import sys
if 'linux' in sys.platform and 'arm' in sys.machine:
    try:
        import RPi.GPIO as GPIO
        print('✓ RPi.GPIO imported successfully')
    except ImportError:
        print('RPi.GPIO not available (expected on non-Raspberry Pi boards)')
    
    try:
        from picamera2 import Picamera2
        print('✓ Picamera2 imported successfully')
    except ImportError:
        print('Picamera2 not available (expected on non-Raspberry Pi boards)')
"
```

## Project Structure

```
nt-rpi-armpi/
├── auto-setup.sh              # Automatic setup script for ARM boards
├── quick-start.sh             # Quick setup + activation helper
├── setup-auto-activation.sh   # Auto-activation setup script
├── requirements.txt           # Python dependencies for ARM boards
├── test_setup.py             # Setup verification script
├── ArmPi_mini.py             # Main application entry point
├── rpc_server.py             # RPC server for remote control
├── Camera.py                 # Camera interface
├── config/                   # Configuration files
│   ├── deviation.yaml        # Servo angle corrections
│   ├── lab_config.yaml       # Color calibration data
│   ├── picking_coordinates.yaml # Pick and place coordinates
│   └── ...
├── functions/                # Robot functions
│   ├── color_detect.py       # Color detection
│   ├── color_sorting.py      # Color sorting
│   ├── face_detect.py        # Face detection
│   ├── color_tracking.py     # Color tracking
│   ├── color_palletizing.py  # Intelligent stacking
│   └── ...
├── armpi_mini_sdk/           # SDK components
│   ├── common_sdk/           # Common utilities
│   └── kinematics_sdk/       # Inverse kinematics
├── board_demo/               # Hardware demos
├── armpi_mini_software/      # GUI software (if available)
└── imx477/                  # Camera-specific code
```

## ArmPi Mini Robot Configuration

### Hardware Setup
1. **Connect the ArmPi Mini robot board** to your ARM board via serial connection
2. **Connect the camera module** (IMX477 or standard Pi camera)
3. **Power the robot** with appropriate power supply
4. **Connect servos** according to the ArmPi Mini documentation

### Camera Configuration
The project supports different camera types:
- **IMX477**: High-resolution camera module (recommended)
- **Standard Pi Camera**: Regular Pi camera module
- **USB Camera**: For testing on other ARM boards

### Robot Configuration
Edit the configuration files in the `config/` directory:
- `deviation.yaml`: Servo angle corrections for your specific robot
- `lab_config.yaml`: Color calibration data for your lighting conditions
- `picking_coordinates.yaml`: Pick and place coordinates for your workspace

## Environment Management

### Manual Activation (Default)
```bash
# Activate environment
source .venv/bin/activate

# Deactivate environment
deactivate
```

### Auto-Activation (Optional)
If you set up auto-activation:
- Environment automatically activates when you enter the project directory
- Environment automatically deactivates when you leave the project directory
- You'll see confirmation messages

### Disabling Auto-Activation
```bash
# Remove the auto-activation function from your shell config
# Edit ~/.bashrc or ~/.zshrc and remove the armpi_mini_auto_activate function
```

## Troubleshooting

### Common Issues

#### 1. YAML Import Error
If you see `AttributeError: module 'yaml' has no attribute 'safe_load'`:
- The updated `yaml_handle.py` should handle this automatically
- If not, install PyYAML: `pip install PyYAML`

#### 2. Camera Not Found
- Check camera connections
- Enable camera in `raspi-config` (Raspberry Pi)
- Test with: `python3 -c "from picamera2 import Picamera2; print('Camera OK')"`

#### 3. Serial Port Access Denied
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Reboot: `sudo reboot`
- Check permissions: `ls -l /dev/ttyAMA0`

#### 4. OpenCV Import Error
- Install system dependencies first
- Try: `pip install opencv-python-headless` for headless systems

#### 5. Board Not Detected
- Check if your board is supported: `cat /proc/device-tree/model`
- Ensure you're running on ARM architecture: `uname -m`

#### 6. Jetson-Specific Issues
- Ensure JetPack is properly installed
- Install jetson-stats: `pip install jetson-stats`
- Use `jtop` to monitor system resources

#### 7. Auto-Activation Issues
- Check your shell type: `echo $SHELL`
- Verify the function was added to your shell config file
- Restart your terminal or reload the config: `source ~/.bashrc`

### Getting Help

1. Check the logs in the `logs/` directory
2. Run the verification script: `python3 test_setup.py`
3. Check system-specific documentation in the `docs/` directory
4. Refer to the ArmPi Mini robot documentation from HiWonder

## Development

### Adding New Dependencies
1. Add to `requirements.txt` with appropriate platform markers
2. Update the setup script if needed
3. Test on different ARM board types

### Testing
```bash
# Run basic tests
python3 test_setup.py

# Test specific components
python3 board_demo/board_test.py
python3 imx477/tests/test_camera.py
```

### Performance Optimization

#### For Raspberry Pi:
- Use `opencv-python-headless` for better performance
- Enable hardware acceleration if available
- Monitor CPU temperature: `vcgencmd measure_temp`

#### For Jetson Nano:
- Use `jetson-stats` to monitor performance
- Enable GPU acceleration for OpenCV
- Monitor power consumption: `jtop`

## License

This project is part of the ArmPi Mini Robot kit from HiWonder. Please refer to the main project documentation for licensing information.

## References

- [HiWonder ArmPi Mini Documentation](https://www.hiwonder.com/)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/jetpack) 