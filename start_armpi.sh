#!/bin/bash

# ArmPi Mini Startup Script
# This script starts the ArmPi Mini system

echo "Starting ArmPi Mini..."

# Change to the correct directory
cd ~/nt_rpi_armpi

# Activate virtual environment if it exists
if [ -d "armpi_env" ]; then
    echo "Activating virtual environment..."
    source armpi_env/bin/activate
fi

# Set Python path
export PYTHONPATH=$PYTHONPATH:/home/pi/nt_rpi_armpi

# Check if hardware is connected
echo "Checking hardware connections..."
if ! lsusb | grep -q "HiWonder\|ArmPi"; then
    echo "Warning: ArmPi board not detected via USB"
    echo "Please check your hardware connections"
fi

# Check camera
if [ ! -e "/dev/video0" ]; then
    echo "Warning: Camera not detected"
    echo "Please check camera connection"
fi

# Start the main program
echo "Starting ArmPi Mini main program..."
echo "Press Ctrl+C to stop"
echo ""
echo "You can access:"
echo "- Camera feed: http://$(hostname -I | awk '{print $1}'):8080"
echo "- RPC server: localhost:5000"
echo ""

python3 ArmPi_mini.py 