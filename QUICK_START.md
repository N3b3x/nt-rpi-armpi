# Quick Start Guide - ArmPi Mini on Raspberry Pi 5

## Immediate Steps to Get Started

### 1. Run the Quick Setup Script
```bash
cd ~/nt_rpi_armpi
chmod +x quick_setup.sh
./quick_setup.sh
```

This script will:
- Update your system
- Install all required dependencies
- Set up Python environment
- Check hardware connections
- Create helper scripts

### 2. Reboot Your Raspberry Pi
```bash
sudo reboot
```

### 3. Start the ArmPi Mini System
```bash
cd ~/nt_rpi_armpi
chmod +x start_armpi.sh
./start_armpi.sh
```

## What You'll Get

After running these commands, you'll have:

- **Main System**: The ArmPi Mini robot control system running
- **Camera Feed**: Accessible at `http://your_pi_ip:8080`
- **RPC Server**: Running on port 5000 for remote control
- **Multiple Functions**: Color detection, tracking, sorting, etc.

## Quick Test Commands

### Test Camera
```bash
python3 Camera.py
```

### Test Basic Movement
```bash
python3 -c "
import sys
sys.path.append('/home/pi/nt_rpi_armpi/')
from kinematics.arm_move_ik import ArmIK
ak = ArmIK()
print('ArmPi initialized successfully')
"
```

### Test Color Detection
```bash
# Start the main system first, then in another terminal:
python3 -c "
import jsonrpc
server = jsonrpc.ServerProxy('http://localhost:5000')
server.loadFunc(2)  # Load color detection
server.startFunc()  # Start it
print('Color detection started')
"
```

## Available Functions

- **Function 1**: Remote Control (motion control)
- **Function 2**: Color Detection
- **Function 3**: Color Sorting
- **Function 4**: Color Tracking
- **Function 5**: Color Palletizing
- **Function 9**: LAB Calibration

## Troubleshooting

### If camera doesn't work:
```bash
sudo usermod -a -G video $USER
sudo reboot
```

### If servos don't respond:
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

### If you get import errors:
```bash
cd ~/nt_rpi_armpi
source armpi_env/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/pi/nt_rpi_armpi
```

## Hardware Checklist

Before starting, make sure:
- [ ] ArmPi Mini board connected via USB
- [ ] Servo motors connected to board
- [ ] Camera connected
- [ ] Power supply connected to ArmPi board

## Next Steps

1. **Calibrate the robot** using Function 9 (LAB calibration)
2. **Test different functions** to understand capabilities
3. **Explore advanced features** like Jacobian-based control
4. **Set up ROS integration** if needed (requires separate planning machine)

## Support

- Check `SETUP_GUIDE.md` for detailed instructions
- Check `JACOBIAN_USAGE_GUIDE.md` for advanced features
- Run with debug logging: `python3 -u ArmPi_mini.py 2>&1 | tee debug.log`

---

**That's it! Your ArmPi Mini should now be running on your Raspberry Pi 5!** 