# 🤖 ArmPi Mini Web Server

A comprehensive web-based control interface for the ArmPi Mini robot that consolidates all functionality into a single, modern web application.

## 🌟 Features

### **Unified Interface**
- **Single Server**: Replaces both the MJPG server (port 8080) and RPC server (port 9030)
- **One Port**: Everything runs on port 8000 for simplicity
- **Modern UI**: Beautiful, responsive web interface with real-time controls

### **Camera & Video**
- **Live Video Streaming**: Real-time camera feed with MJPEG streaming
- **Snapshot Capture**: Take and download snapshots directly from the browser
- **Demo Mode**: Displays demo frames when camera hardware is not available

### **Robot Control**
- **Function Management**: Load, start, and stop robot functions (color detection, tracking, sorting, face detection)
- **Servo Control**: Real-time slider controls for individual servo positioning
- **Color Selection**: Visual color picker for tracking and sorting operations
- **Motor Control**: Brush motor control via JSON-RPC

### **Advanced Features**
- **Battery Monitoring**: Real-time battery voltage display
- **Command Logging**: Real-time command history and status updates
- **Status Updates**: Automatic system status monitoring
- **Demo Mode**: Full functionality simulation when robot hardware is unavailable

## 🚀 Quick Start

### Method 1: Using the Startup Script (Recommended)
```bash
./start_web_server.sh
```

### Method 2: Direct Python Execution
```bash
python3 web_server.py
```

### Method 3: Installing Dependencies Manually
```bash
pip3 install -r requirements.txt
python3 web_server.py
```

### Method 4: System Service (Auto-start on boot)
```bash
./install_service.sh
```

## 📁 Dynamic Path Detection

The web server automatically detects its location and adjusts paths accordingly:

- ✅ **Works from any directory** - no hardcoded paths
- ✅ **Automatic module discovery** - finds robot modules relative to script location
- ✅ **Legacy compatibility** - supports old `/home/pi/ArmPi_mini/` structure
- ✅ **Service installer** - generates correct systemd service files
- ✅ **Portable** - can be moved anywhere on the filesystem

## 🌐 Access the Interface

Once started, access the web interface at:
- **Local**: http://localhost:8000
- **Network**: http://[robot-ip]:8000

## 📱 Web Interface Overview

### 🎥 Video Section
- **Live Camera Feed**: Real-time video streaming from the robot camera
- **Snapshot Button**: Capture and download images
- **Automatic Fallback**: Shows demo mode when camera is unavailable

### 🎮 Control Panels

#### Robot Functions
- **Color Detection**: Detect and identify colored objects
- **Color Tracking**: Follow colored objects with the camera
- **Color Sorting**: Sort objects by color
- **Face Detection**: Detect and track human faces
- **Start/Stop Controls**: Manage function execution

#### Color Selection
- **Visual Color Picker**: Click color buttons to set target colors
- **Real-time Updates**: Immediate color target changes
- **Multiple Colors**: Red, Green, Blue, Yellow, Purple, Orange

#### Servo Control
- **Interactive Sliders**: Real-time servo positioning
- **Visual Feedback**: Live angle display
- **Smooth Control**: 1000ms transition time for smooth movement

#### Command Log
- **Real-time Logging**: See all commands and responses
- **Timestamps**: Track when commands were executed
- **Scrollable History**: Review previous operations

## 🔧 API Endpoints

### Web Routes
- `GET /` - Main web interface
- `GET /video_feed` - MJPEG video stream
- `GET /snapshot` - Capture single image
- `POST /rpc` - JSON-RPC command interface
- `GET /api/robot_status` - System status JSON

### JSON-RPC Methods
All original RPC methods are supported:

#### Robot Functions
- `LoadFunc(function_id)` - Load robot function
- `StartFunc()` - Start current function
- `StopFunc()` - Stop current function
- `UnloadFunc()` - Unload current function

#### Servo Control
- `SetPWMServo(time, servo1, angle1, servo2, angle2, ...)` - Control servos
- `SetBusServoPulse(time, count, servo1, pulse1, ...)` - Bus servo control

#### Color Operations
- `ColorTracking(color)` - Set color tracking target
- `ColorSorting(color)` - Set color sorting target
- `ColorPalletizing(color)` - Set color palletizing target
- `SetLABValue(lab_values)` - Set color thresholds
- `GetLABValue()` - Get current color thresholds

#### System Information
- `GetBatteryVoltage()` - Get battery status
- `GetSystemInfo()` - Get comprehensive system information
- `Heartbeat()` - Keep connection alive

## 🔄 Migration from Separate Servers

### Before (Two Servers)
```bash
# Terminal 1
python3 mjpg_server.py  # Port 8080

# Terminal 2  
python3 rpc_server.py   # Port 9030
```

### After (Single Server)
```bash
# Single Terminal
./start_web_server.sh   # Port 8000
```

### Port Mapping
- **Old MJPG**: `:8080` → **New**: `:8000/video_feed`
- **Old RPC**: `:9030` → **New**: `:8000/rpc`
- **New Web UI**: `:8000/` (main interface)

## 🛠️ Technical Details

### Dependencies
- **Flask**: Web framework for HTTP server
- **OpenCV**: Camera and image processing
- **NumPy**: Numerical operations
- **jsonrpc2**: JSON-RPC protocol implementation
- **Robot Modules**: ArmPi Mini specific hardware control

### Demo Mode
When robot hardware is not available, the server automatically switches to demo mode:
- **Demo Video**: Generated frames with timestamp
- **Simulated Responses**: All RPC calls return success with demo data
- **Full UI**: Complete interface functionality for testing

### Performance
- **Video Streaming**: ~30 FPS with quality optimization
- **Response Time**: Sub-100ms for most RPC calls
- **Memory Usage**: Efficient frame buffering
- **Threading**: Separate threads for video and control

## 🎨 Customization

### Styling
The interface uses modern CSS with:
- **Gradient Backgrounds**: Purple/blue theme
- **Responsive Design**: Works on desktop and mobile
- **Smooth Animations**: Hover effects and transitions
- **Glass Morphism**: Modern UI design trends

### Adding Functions
To add new robot functions:

1. **Add RPC Method**:
```python
@_rpc.register
def MyNewFunction(param1, param2):
    if not ROBOT_AVAILABLE:
        return (True, "Demo: New function executed", 'MyNewFunction')
    # Real implementation here
    return (True, "Success", 'MyNewFunction')
```

2. **Add UI Button**:
```javascript
async function myNewFunction() {
    await sendRPC('MyNewFunction', ['param1', 'param2']);
}
```

3. **Add HTML**:
```html
<button class="btn btn-primary" onclick="myNewFunction()">My Function</button>
```

## 🐛 Troubleshooting

### Common Issues

**"Camera Not Available"**
- Check camera connection
- Ensure camera is not used by another process
- Try demo mode for testing interface

**"RPC Error"**
- Check if robot modules are properly installed
- Verify hardware connections
- Use demo mode to test interface functionality

**"Port Already in Use"**
- Stop any existing servers on port 8000
- Check for other Flask applications
- Use `lsof -i :8000` to find conflicting processes

### Log Analysis
Check the console output for:
- **"Robot modules not available"**: Hardware/software issue
- **"Demo mode"**: Running without robot hardware
- **"Camera opened"**: Successful camera initialization

## 🔒 Security Notes

- **Local Network**: Server binds to 0.0.0.0 for network access
- **No Authentication**: Basic setup without user authentication
- **Direct Hardware**: Commands directly control robot hardware

For production use, consider adding:
- User authentication
- HTTPS encryption
- Command rate limiting
- Input validation

## 📞 Support

For issues or questions:
1. Check the command log in the web interface
2. Review console output for error messages
3. Try demo mode to isolate hardware issues
4. Verify all dependencies are installed

## 🎯 Future Enhancements

Potential improvements:
- **Multi-camera Support**: Multiple video streams
- **Recording**: Video recording and playback
- **Mobile App**: Native mobile interface
- **Voice Control**: Speech recognition integration
- **Advanced AI**: Computer vision pipelines
- **Remote Access**: Secure external access