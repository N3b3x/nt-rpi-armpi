#!/usr/bin/env bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to detect ARM-based single-board computer
detect_board() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [[ -f "/etc/os-release" ]]; then
            . /etc/os-release
            if [[ "$(uname -m)" == "armv7l" || "$(uname -m)" == "aarch64" ]]; then
                # Check for specific board types
                if [[ -f "/proc/device-tree/model" ]]; then
                    BOARD_MODEL=$(cat /proc/device-tree/model)
                    if [[ "$BOARD_MODEL" == *"Raspberry Pi"* ]]; then
                        BOARD_TYPE="raspberry_pi"
                        print_status "Detected Raspberry Pi: $BOARD_MODEL"
                    elif [[ "$BOARD_MODEL" == *"Jetson"* ]]; then
                        BOARD_TYPE="jetson"
                        print_status "Detected NVIDIA Jetson: $BOARD_MODEL"
                    elif [[ "$BOARD_MODEL" == *"Orange Pi"* ]]; then
                        BOARD_TYPE="orange_pi"
                        print_status "Detected Orange Pi: $BOARD_MODEL"
                    elif [[ "$BOARD_MODEL" == *"Rock Pi"* ]]; then
                        BOARD_TYPE="rock_pi"
                        print_status "Detected Rock Pi: $BOARD_MODEL"
                    else
                        BOARD_TYPE="generic_arm"
                        print_status "Detected generic ARM board: $BOARD_MODEL"
                    fi
                else
                    BOARD_TYPE="generic_arm"
                    print_status "Detected generic ARM system"
                fi
            else
                print_error "This script is designed for ARM-based single-board computers only."
                print_error "Detected architecture: $(uname -m)"
                print_error "Please run this on a Raspberry Pi, Jetson Nano, or similar ARM board."
                exit 1
            fi
        else
            print_error "Could not detect OS type. This script is for ARM-based boards only."
            exit 1
        fi
    else
        print_error "This script is designed for Linux-based ARM single-board computers only."
        print_error "Detected OS: $OSTYPE"
        exit 1
    fi
}

# Function to install system dependencies
install_system_deps() {
    print_status "Installing system dependencies for $BOARD_TYPE..."
    
    case $BOARD_TYPE in
        "raspberry_pi")
            print_status "Installing Raspberry Pi specific dependencies..."
            sudo apt-get update
            sudo apt-get install -y \
                python3-pip \
                python3-venv \
                python3-dev \
                libatlas-base-dev \
                libhdf5-dev \
                libhdf5-serial-dev \
                libatlas-base-dev \
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
                libzmq3-dev
            ;;
        "jetson")
            print_status "Installing Jetson specific dependencies..."
            sudo apt-get update
            sudo apt-get install -y \
                python3-pip \
                python3-venv \
                python3-dev \
                libatlas-base-dev \
                libhdf5-dev \
                libhdf5-serial-dev \
                libatlas-base-dev \
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
                libzmq3-dev
            ;;
        "orange_pi"|"rock_pi"|"generic_arm")
            print_status "Installing generic ARM board dependencies..."
            sudo apt-get update
            sudo apt-get install -y \
                python3-pip \
                python3-venv \
                python3-dev \
                libatlas-base-dev \
                libhdf5-dev \
                libhdf5-serial-dev \
                libatlas-base-dev \
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
                libzmq3-dev
            ;;
        *)
            print_error "Unsupported board type: $BOARD_TYPE"
            exit 1
            ;;
    esac
}

# Function to setup Python environment
setup_python_env() {
    ENV_NAME=${1:-.venv}
    print_status "Creating virtual environment '$ENV_NAME'"
    
    # Create virtual environment
    python3 -m venv "$ENV_NAME"
    
    # Activate virtual environment
    source "$ENV_NAME/bin/activate"
    
    # Upgrade pip
    print_status "Upgrading pip..."
    pip install --upgrade pip
    
    # Install Python dependencies
    print_status "Installing Python dependencies..."
    
    # Handle remote control dependencies for ARM64
    print_status "Setting up remote control dependencies..."
    source "$ENV_NAME/bin/activate"
    
    # Install system PyZMQ (most reliable for ARM64 + Python 3.11)
    print_status "Installing PyZMQ via system package (recommended for ARM64)..."
    sudo apt-get install -y python3-zmq
    
    # Verify PyZMQ installation
    if python3 -c "import zmq; print('PyZMQ version:', zmq.__version__)" 2>/dev/null; then
        print_success "PyZMQ installed successfully via system package"
    else
        print_warning "System PyZMQ installation failed. Remote control may not work."
        print_warning "This is expected on some ARM64 + Python 3.11 configurations."
    fi
    
    # Note: jsonrpc2-zeromq removed from requirements.txt due to compilation issues
    # The project uses HTTP-based RPC which works without ZeroMQ
    
    # Install remaining dependencies (excluding PyZMQ which is already installed)
    print_status "Installing remaining Python dependencies..."
    pip install -r requirements.txt
    
    # Install board-specific dependencies
    case $BOARD_TYPE in
        "raspberry_pi")
            print_status "Installing Raspberry Pi specific Python packages..."
            pip install picamera2
            ;;
        "jetson")
            print_status "Installing Jetson specific Python packages..."
            pip install jetson-stats
            # Jetson may need specific OpenCV version
            pip install opencv-python-headless
            ;;
        "orange_pi"|"rock_pi"|"generic_arm")
            print_status "Installing generic ARM board Python packages..."
            # Try to install picamera2 if available
            pip install picamera2 2>/dev/null || print_warning "picamera2 not available for this board"
            ;;
    esac
}

# Function to setup project structure and permissions
setup_project() {
    print_status "Setting up project structure..."
    
    # Create necessary directories if they don't exist
    mkdir -p config
    mkdir -p logs
    mkdir -p data
    
    # Set up permissions for serial access
    print_status "Setting up serial port permissions..."
    sudo usermod -a -G dialout $USER
    
    # Set permissions for common serial ports
    sudo chmod 666 /dev/ttyAMA0 2>/dev/null || true
    sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true
    sudo chmod 666 /dev/ttyS0 2>/dev/null || true
    
    # Enable camera if on Raspberry Pi
    if [[ "$BOARD_TYPE" == "raspberry_pi" ]]; then
        print_status "Enabling camera interface..."
        sudo raspi-config nonint do_camera 0 2>/dev/null || print_warning "Could not enable camera via raspi-config"
    fi
    
    # Enable I2C and SPI if on Raspberry Pi
    if [[ "$BOARD_TYPE" == "raspberry_pi" ]]; then
        print_status "Enabling I2C and SPI interfaces..."
        sudo raspi-config nonint do_i2c 0 2>/dev/null || print_warning "Could not enable I2C via raspi-config"
        sudo raspi-config nonint do_spi 0 2>/dev/null || print_warning "Could not enable SPI via raspi-config"
    fi
}

# Function to verify installation
verify_installation() {
    print_status "Verifying installation..."
    
    source "$ENV_NAME/bin/activate"
    
    # Test basic imports
    python3 -c "
import sys
print('Python version:', sys.version)

try:
    import numpy as np
    print('✓ NumPy:', np.__version__)
except ImportError as e:
    print('✗ NumPy import failed:', e)

try:
    import cv2
    print('✓ OpenCV:', cv2.__version__)
except ImportError as e:
    print('✗ OpenCV import failed:', e)

try:
    import yaml
    print('✓ PyYAML imported successfully')
except ImportError as e:
    print('✗ PyYAML import failed:', e)

try:
    import serial
    print('✓ PySerial imported successfully')
except ImportError as e:
    print('✗ PySerial import failed:', e)

try:
    import matplotlib
    print('✓ Matplotlib:', matplotlib.__version__)
except ImportError as e:
    print('✗ Matplotlib import failed:', e)

# Test board-specific imports
if '$BOARD_TYPE' == 'raspberry_pi':
    try:
        import RPi.GPIO as GPIO
        print('✓ RPi.GPIO imported successfully')
    except ImportError as e:
        print('✗ RPi.GPIO import failed:', e)
    
    try:
        from picamera2 import Picamera2
        print('✓ Picamera2 imported successfully')
    except ImportError as e:
        print('✗ Picamera2 import failed:', e)
elif '$BOARD_TYPE' == 'jetson':
    try:
        import jtop
        print('✓ Jetson stats imported successfully')
    except ImportError as e:
        print('✗ Jetson stats import failed:', e)

print('\\nInstallation verification complete!')
"
}

# Function to display board information
display_board_info() {
    print_status "Board Information:"
    echo "Board Type: $BOARD_TYPE"
    echo "Architecture: $(uname -m)"
    echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
    echo "Kernel: $(uname -r)"
    echo "CPU: $(cat /proc/cpuinfo | grep 'Model name' | head -1 | cut -d':' -f2 | xargs)"
    echo "Memory: $(free -h | grep Mem | awk '{print $2}')"
    
    if [[ "$BOARD_TYPE" == "raspberry_pi" ]]; then
        echo "Raspberry Pi Model: $(cat /proc/device-tree/model)"
        echo "Raspberry Pi Revision: $(cat /proc/cpuinfo | grep Revision | head -1 | cut -d':' -f2 | xargs)"
    elif [[ "$BOARD_TYPE" == "jetson" ]]; then
        echo "Jetson Model: $(cat /proc/device-tree/model)"
    fi
    echo ""
}

# Main setup function
main() {
    print_status "Starting ArmPi Mini Robot setup for ARM-based single-board computers..."
    
    # Detect board type
    detect_board
    
    # Display board information
    display_board_info
    
    # Install system dependencies
    install_system_deps
    
    # Setup Python environment
    setup_python_env "$1"
    
    # Setup project structure
    setup_project
    
    # Verify installation
    verify_installation
    
    print_success "Setup complete for $BOARD_TYPE!"
    print_status "To activate the environment, run:"
    echo "source $1/bin/activate"
    echo ""
    print_status "To run the main application:"
    echo "python3 ArmPi_mini.py"
    echo ""
    
    print_warning "IMPORTANT: You need to reboot for serial port permissions to take effect."
    print_warning "Run: sudo reboot"
    echo ""
    
    print_status "For more information, see SETUP.md"
}

# Check if script is run with correct arguments
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 [environment_name]"
    echo "  environment_name: Name of the virtual environment (default: .venv)"
    echo ""
    echo "This script is designed for ARM-based single-board computers:"
    echo "  - Raspberry Pi (all models)"
    echo "  - NVIDIA Jetson Nano/AGX/Xavier"
    echo "  - Orange Pi"
    echo "  - Rock Pi"
    echo "  - Other ARM-based boards"
    echo ""
    echo "This script will:"
    echo "  1. Detect your ARM board type"
    echo "  2. Install system dependencies"
    echo "  3. Create a Python virtual environment"
    echo "  4. Install Python dependencies"
    echo "  5. Setup project structure and permissions"
    echo "  6. Verify the installation"
    exit 0
fi

# Run main setup
main "$1"
