#!/bin/bash

# ArmPi Mini Web Server Startup Script

echo "================================================"
echo "🤖 ArmPi Mini Robot Web Server"
echo "================================================"
echo ""

# Set the working directory to the script location (repository root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "📍 Repository location: $SCRIPT_DIR"

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3 to continue."
    exit 1
fi

echo "✅ Python 3 found: $(python3 --version)"

# Check if required dependencies are installed
echo "🔍 Checking dependencies..."

# Try to import required modules
python3 -c "
import sys
missing = []
try:
    import flask
except ImportError:
    missing.append('flask')
try:
    import cv2
except ImportError:
    missing.append('opencv-python')
try:
    import numpy
except ImportError:
    missing.append('numpy')
try:
    import jsonrpc2
except ImportError:
    missing.append('jsonrpc2')

if missing:
    print(f'❌ Missing dependencies: {missing}')
    print('📦 Installing missing dependencies...')
    import subprocess
    subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + missing)
    print('✅ Dependencies installed successfully!')
else:
    print('✅ All dependencies are available')
"

if [ $? -ne 0 ]; then
    echo "❌ Failed to check/install dependencies"
    exit 1
fi

echo ""
echo "🚀 Starting ArmPi Mini Web Server..."
echo "📱 Access the robot control interface at:"
echo "   http://localhost:8000"
echo "   http://$(hostname -I | awk '{print $1}'):8000"
echo ""
echo "⏹️  Press Ctrl+C to stop the server"
echo ""

# Start the web server
python3 web_server.py