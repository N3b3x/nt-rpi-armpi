# 🤖 ArmPi Mini Robot

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/Platform-ARM%20Boards-green.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen.svg)](https://github.com/your-username/nt-rpi-armpi)

> **Advanced robotic arm control system for ARM-based single-board computers**

The ArmPi Mini Robot is a 4 DOF robotic arm kit from HiWonder designed to work with Raspberry Pi and similar ARM boards. This project provides a complete software stack for controlling the ArmPi Mini robot with advanced features including computer vision, color detection, face recognition, and intelligent object manipulation.

## 📋 Table of Contents

- [Features](#-features)
- [Supported Hardware](#-supported-hardware)
- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Usage](#-usage)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Development](#-development)
- [Contributing](#-contributing)
- [License](#-license)

## ✨ Features

### 🤖 Core Robot Functions
- **Inverse Kinematics**: Advanced mathematical calculations for precise arm positioning
- **Servo Control**: Smooth and accurate servo motor control with angle compensation
- **Serial Communication**: Reliable communication with the robot control board
- **Real-time Control**: Responsive control system for immediate robot actions

### 👁️ Computer Vision
- **Color Detection**: Advanced color recognition and tracking
- **Face Detection & Recognition**: AI-powered facial recognition system
- **Object Tracking**: Real-time object tracking and following
- **Image Processing**: High-performance image processing with OpenCV

### 🎯 Advanced Capabilities
- **Color Sorting**: Intelligent sorting of objects by color
- **Color Palletizing**: Automated stacking and organizing of objects
- **Position Detection**: Precise object position detection and mapping
- **Remote Control**: Web-based remote control interface
- **Gesture Recognition**: Hand gesture detection and interpretation

### 🔧 Technical Features
- **Multi-Platform Support**: Works on Raspberry Pi, Jetson Nano, and other ARM boards
- **Modular Architecture**: Clean, maintainable code structure
- **Configuration Management**: Flexible YAML-based configuration system
- **Error Handling**: Robust error handling and recovery mechanisms
- **Performance Optimization**: Optimized for ARM-based systems

## 🖥️ Supported Hardware

This project is specifically designed for ARM-based single-board computers:

| Board Type | Models | Status | Notes |
|------------|--------|--------|-------|
| **Raspberry Pi** | Pi 3, Pi 4, Pi Zero W, Pi 400 | ✅ Full Support | Recommended platform |
| **NVIDIA Jetson** | Nano, AGX Xavier, Xavier NX | ✅ Full Support | GPU acceleration available |
| **Orange Pi** | All models | ✅ Full Support | Compatible with most features |
| **Rock Pi** | All models | ✅ Full Support | Good performance |
| **Other ARM** | aarch64/armv7l boards | ⚠️ Limited Support | Basic features only |

### System Requirements
- **OS**: Linux-based (Raspberry Pi OS, Ubuntu, Debian)
- **Python**: 3.8 or higher
- **RAM**: 2GB minimum (4GB recommended)
- **Storage**: 8GB minimum free space
- **Camera**: Compatible camera module (IMX477 recommended)
- **Serial**: USB or GPIO serial connection

## 🚀 Quick Start

### Option 1: One-Click Setup (Recommended)

```bash
# Clone the repository
git clone https://github.com/your-username/nt-rpi-armpi.git
cd nt-rpi-armpi

# Make scripts executable
chmod +x auto-setup.sh quick-start.sh

# Run the quick start script
./quick-start.sh
```

### Option 2: Manual Setup

```bash
# Clone the repository
git clone https://github.com/your-username/nt-rpi-armpi.git
cd nt-rpi-armpi

# Run the automatic setup
chmod +x auto-setup.sh
./auto-setup.sh

# Activate the environment
source .venv/bin/activate

# Run the main application
python3 ArmPi_mini.py
```

### Option 3: Auto-Activation Setup

```bash
# Complete setup with auto-activation
./auto-setup.sh
chmod +x setup-auto-activation.sh
./setup-auto-activation.sh

# Reload shell configuration
source ~/.bashrc  # or ~/.zshrc for Zsh
```

## 📦 Installation

### Prerequisites

#### For Raspberry Pi
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install essential packages
sudo apt-get install -y python3-pip python3-venv python3-dev git
```

#### For Jetson Nano
```bash
# Ensure JetPack is installed
# Install additional dependencies
sudo apt-get install -y python3-pip python3-venv python3-dev git
```

#### For Other ARM Boards
```bash
# Install base dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv python3-dev git
```

### Step-by-Step Installation

#### 1. System Dependencies

**Raspberry Pi:**
```bash
sudo apt-get install -y \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libjasper-dev \
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
    pkg-config
```

**Jetson Nano:**
```bash
sudo apt-get install -y \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libjasper-dev \
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
    pkg-config
```

#### 2. Python Environment

```bash
# Create virtual environment
python3 -m venv .venv

# Activate environment
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install Python dependencies
pip install -r requirements.txt
```

#### 3. Hardware Setup

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Set permissions for serial ports
sudo chmod 666 /dev/ttyAMA0 2>/dev/null || true
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true
sudo chmod 666 /dev/ttyS0 2>/dev/null || true

# Enable interfaces (Raspberry Pi only)
if [[ -f "/proc/device-tree/model" ]] && grep -q "Raspberry Pi" /proc/device-tree/model; then
    sudo raspi-config nonint do_camera 0
    sudo raspi-config nonint do_i2c 0
    sudo raspi-config nonint do_spi 0
fi

# Reboot for changes to take effect
sudo reboot
```

#### 4. Verification

```bash
# Activate environment
source .venv/bin/activate

# Run verification script
python3 test_setup.py
```

## 🎮 Usage

### Basic Usage

```bash
# Activate the environment
source .venv/bin/activate

# Run the main application
python3 ArmPi_mini.py
```

### Available Functions

| Function | Description | Command |
|----------|-------------|---------|
| **Color Detection** | Detect and identify colors | `python3 functions/color_detect.py` |
| **Color Sorting** | Sort objects by color | `python3 functions/color_sorting.py` |
| **Face Detection** | Detect faces in camera feed | `python3 functions/face_detect.py` |
| **Face Recognition** | Recognize known faces | `python3 functions/face_recognition.py` |
| **Color Tracking** | Track colored objects | `python3 functions/color_tracking.py` |
| **Color Palletizing** | Intelligent object stacking | `python3 functions/color_palletizing.py` |
| **Remote Control** | Web-based remote control | `python3 rpc_server.py` |

### Demo Scripts

```bash
# Hardware demos
python3 board_demo/board_test.py
python3 board_demo/gripper_test.py
python3 board_demo/rgb_control_demo.py

# Camera demos
python3 imx477/tests/test_camera.py
```

## ⚙️ Configuration

### Configuration Files

The project uses YAML configuration files located in the `config/` directory:

| File | Purpose | Description |
|------|---------|-------------|
| `deviation.yaml` | Servo Calibration | Angle corrections for each servo |
| `lab_config.yaml` | Color Calibration | Color detection parameters |
| `picking_coordinates.yaml` | Workspace Setup | Pick and place coordinates |
| `table_height.yaml` | Height Settings | Table and object heights |
| `stacking_height.yaml` | Stacking Parameters | Object stacking configuration |

### Hardware Configuration

#### Camera Setup
```yaml
# config/camera_config.yaml
camera:
  type: "imx477"  # or "standard_pi", "usb"
  resolution: [1920, 1080]
  fps: 30
  exposure: "auto"
```

#### Servo Configuration
```yaml
# config/servo_config.yaml
servos:
  servo3:
    range: [500, 2500]
    angle_range: [0, 180]
    offset: 0
  servo4:
    range: [500, 2500]
    angle_range: [0, 180]
    offset: 0
  # ... more servos
```

### Environment Variables

```bash
# Optional environment variables
export ARMPI_DEBUG=1          # Enable debug logging
export ARMPI_CAMERA_DEVICE=0  # Camera device number
export ARMPI_SERIAL_PORT=/dev/ttyAMA0  # Serial port
```

## 🔧 Troubleshooting

### Common Issues

#### 1. YAML Import Error
```bash
# Error: AttributeError: module 'yaml' has no attribute 'safe_load'
# Solution: The updated yaml_handle.py should handle this automatically
# If not, install PyYAML manually:
pip install PyYAML
```

#### 2. Camera Not Found
```bash
# Check camera connections
ls -l /dev/video*

# Enable camera (Raspberry Pi)
sudo raspi-config nonint do_camera 0

# Test camera
python3 -c "from picamera2 import Picamera2; print('Camera OK')"
```

#### 3. Serial Port Access Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set permissions
sudo chmod 666 /dev/ttyAMA0

# Reboot
sudo reboot
```

#### 4. OpenCV Import Error
```bash
# Install system dependencies first
sudo apt-get install -y libopencv-dev

# Try headless version
pip install opencv-python-headless
```

#### 5. Board Detection Issues
```bash
# Check board type
cat /proc/device-tree/model

# Check architecture
uname -m

# Verify ARM architecture
if [[ "$(uname -m)" == "armv7l" || "$(uname -m)" == "aarch64" ]]; then
    echo "ARM architecture detected"
else
    echo "Non-ARM architecture - not supported"
fi
```

### Performance Issues

#### Raspberry Pi Optimization
```bash
# Monitor temperature
vcgencmd measure_temp

# Enable hardware acceleration
sudo raspi-config nonint do_memory_split 128

# Use headless OpenCV
pip install opencv-python-headless
```

#### Jetson Nano Optimization
```bash
# Monitor performance
jtop

# Check GPU usage
nvidia-smi

# Monitor power
sudo tegrastats
```

### Getting Help

1. **Check Logs**: Look in the `logs/` directory for error messages
2. **Run Tests**: Execute `python3 test_setup.py` for diagnostics
3. **Documentation**: Check the `docs/` directory for detailed guides
4. **Issues**: Report problems on the GitHub issues page

## 🛠️ Development

### Project Structure

```
nt-rpi-armpi/
├── 📁 action_groups/          # Robot action sequences
├── 📁 armpi_mini_demo/        # Demo applications
├── 📁 armpi_mini_sdk/         # SDK components
│   ├── 📁 common_sdk/         # Common utilities
│   └── 📁 kinematics_sdk/     # Inverse kinematics
├── 📁 armpi_mini_software/    # GUI software
├── 📁 board_demo/             # Hardware demos
├── 📁 CAD/                    # 3D models and designs
├── 📁 CameraCalibration/      # Camera calibration tools
├── 📁 config/                 # Configuration files
├── 📁 docs/                   # Documentation
├── 📁 functions/              # Robot functions
├── 📁 hiwonder-toolbox/       # HiWonder utilities
├── 📁 imx477/                 # Camera-specific code
├── 📁 tests/                  # Test files
├── 🔧 auto-setup.sh           # Automatic setup script
├── 🔧 quick-start.sh          # Quick start helper
├── 🔧 setup-auto-activation.sh # Auto-activation setup
├── 📋 requirements.txt        # Python dependencies
├── 🧪 test_setup.py          # Setup verification
├── 🤖 ArmPi_mini.py          # Main application
├── 🌐 rpc_server.py          # Remote control server
└── 📷 Camera.py              # Camera interface
```

### Adding New Features

#### 1. New Robot Function
```python
# functions/new_function.py
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from armpi_mini_sdk.common_sdk.common import Board
from armpi_mini_sdk.kinematics_sdk.my_kinematics.arm_move_ik import ArmIK

class NewFunction:
    def __init__(self):
        self.board = Board()
        self.arm = ArmIK()
        self.arm.board = self.board
    
    def execute(self):
        # Your new function logic here
        pass

if __name__ == "__main__":
    function = NewFunction()
    function.execute()
```

#### 2. New Configuration
```yaml
# config/new_config.yaml
new_feature:
  enabled: true
  parameters:
    param1: "value1"
    param2: "value2"
```

#### 3. Testing
```bash
# Run all tests
python3 test_setup.py

# Run specific tests
python3 -m pytest tests/

# Test new function
python3 functions/new_function.py
```

### Code Style

- **Python**: Follow PEP 8 guidelines
- **Documentation**: Use docstrings for all functions
- **Error Handling**: Implement proper exception handling
- **Logging**: Use the logging module for debug information

## 🤝 Contributing

We welcome contributions! Please follow these steps:

### 1. Fork the Repository
```bash
git clone https://github.com/your-username/nt-rpi-armpi.git
cd nt-rpi-armpi
```

### 2. Create a Feature Branch
```bash
git checkout -b feature/your-feature-name
```

### 3. Make Your Changes
- Follow the coding style guidelines
- Add tests for new features
- Update documentation

### 4. Test Your Changes
```bash
# Run the test suite
python3 test_setup.py

# Test on different ARM boards if possible
```

### 5. Submit a Pull Request
- Provide a clear description of your changes
- Include any relevant issue numbers
- Ensure all tests pass

### Development Setup

```bash
# Clone the repository
git clone https://github.com/your-username/nt-rpi-armpi.git
cd nt-rpi-armpi

# Create development environment
python3 -m venv .venv-dev
source .venv-dev/bin/activate

# Install development dependencies
pip install -r requirements.txt
pip install pytest black flake8

# Run code formatting
black .

# Run linting
flake8 .

# Run tests
pytest tests/
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

The ArmPi Mini Robot hardware and original software are products of HiWonder. This repository contains enhanced software for the ArmPi Mini Robot kit.

## 🙏 Acknowledgments

- **HiWonder** for the ArmPi Mini Robot hardware and original software
- **Raspberry Pi Foundation** for the Raspberry Pi platform
- **NVIDIA** for the Jetson platform
- **OpenCV** community for computer vision capabilities
- **All contributors** who have helped improve this project

## 📞 Support

### Getting Help

- **Documentation**: Check the [SETUP.md](SETUP.md) for detailed setup instructions
- **Issues**: Report bugs and request features on [GitHub Issues](https://github.com/your-username/nt-rpi-armpi/issues)
- **Discussions**: Join the conversation on [GitHub Discussions](https://github.com/your-username/nt-rpi-armpi/discussions)
- **Wiki**: Check the [Wiki](https://github.com/your-username/nt-rpi-armpi/wiki) for additional resources

### Community

- **Discord**: Join our [Discord server](https://discord.gg/your-server)
- **Forum**: Visit our [community forum](https://forum.your-domain.com)
- **YouTube**: Watch tutorials on our [YouTube channel](https://youtube.com/your-channel)

## 📊 Project Status

| Component | Status | Version |
|-----------|--------|---------|
| Core Robot Control | ✅ Stable | 1.0.0 |
| Computer Vision | ✅ Stable | 1.0.0 |
| Color Detection | ✅ Stable | 1.0.0 |
| Face Recognition | ✅ Stable | 1.0.0 |
| Remote Control | ✅ Stable | 1.0.0 |
| Documentation | ✅ Complete | 1.0.0 |
| Tests | ✅ Comprehensive | 1.0.0 |

---

<div align="center">

**Made with ❤️ for the robotics community**

[![GitHub stars](https://img.shields.io/github/stars/your-username/nt-rpi-armpi?style=social)](https://github.com/your-username/nt-rpi-armpi/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/your-username/nt-rpi-armpi?style=social)](https://github.com/your-username/nt-rpi-armpi/network/members)
[![GitHub issues](https://img.shields.io/github/issues/your-username/nt-rpi-armpi)](https://github.com/your-username/nt-rpi-armpi/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/your-username/nt-rpi-armpi)](https://github.com/your-username/nt-rpi-armpi/pulls)

</div> 
