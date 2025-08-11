#!/usr/bin/env python3
# encoding:utf-8

import os
import sys
import json
import time
import logging
import threading
import numpy as np
from io import BytesIO

# Add the paths to the common and kinematics modules dynamically
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'common_sdk'))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'kinematics_sdk'))
sys.path.append(current_dir)  # Add current directory to Python path

# Legacy path support (if the old structure exists)
legacy_path = '/home/pi/ArmPi_mini/'
if os.path.exists(legacy_path) and legacy_path not in sys.path:
    sys.path.append(legacy_path)

import cv2
try:
    import functions.running as running
    import functions.lab_adjust as lab_adjust
    import functions.color_sorting as color_sorting
    import functions.color_detect as color_detect
    import functions.color_tracking as color_tracking
    import functions.color_palletizing as color_palletizing
    from my_kinematics.arm_move_ik import *
    from Camera import Camera
    ROBOT_AVAILABLE = True
except ImportError as e:
    print(f"Robot modules not available: {e}")
    ROBOT_AVAILABLE = False

from flask import Flask, Response, request, jsonify, render_template_string, send_from_directory
from werkzeug.serving import run_simple
from jsonrpc2 import JsonRpc

# Initialize Flask app
app = Flask(__name__)

# Initialize JSON-RPC 2.0 router
_rpc = JsonRpc()

# Global variables for camera and robot control
camera = None
img_show = None
quality = (int(cv2.IMWRITE_JPEG_QUALITY), 70)
HWSONAR = None
QUEUE = None

# RPC Error codes
__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

def initialize_robot():
    """Initialize robot hardware and functions"""
    global camera, board, AK
    
    if not ROBOT_AVAILABLE:
        print("Robot modules not available - running in demo mode")
        # Create a demo camera that generates test frames
        threading.Thread(target=create_demo_frames, daemon=True).start()
        return
    
    try:
        camera = Camera()
        camera.camera_open()
        
        # Initialize board and AK if available
        try:
            set_board()
        except:
            print("Warning: Could not initialize robot board")
        
        # Start camera frame update thread
        threading.Thread(target=update_camera_frame, daemon=True).start()
        
    except Exception as e:
        print(f"Error initializing robot: {e}")
        # Fallback to demo mode
        threading.Thread(target=create_demo_frames, daemon=True).start()

def create_demo_frames():
    """Create demo frames when camera is not available"""
    global img_show
    while True:
        try:
            # Create a simple demo frame
            demo_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            demo_frame[:] = (50, 50, 100)  # Dark blue background
            
            # Add some text
            cv2.putText(demo_frame, "ArmPi Mini Robot", (150, 200), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
            cv2.putText(demo_frame, "Demo Mode", (250, 250), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(demo_frame, f"Time: {time.strftime('%H:%M:%S')}", (200, 300), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
            
            img_show = demo_frame
            time.sleep(0.1)  # 10 FPS for demo
        except Exception as e:
            print(f"Error creating demo frame: {e}")
            time.sleep(1)

def update_camera_frame():
    """Update global img_show variable with camera frames"""
    global img_show, camera
    while True:
        try:
            if camera and camera.frame is not None:
                img_show = camera.frame.copy()
            time.sleep(0.033)  # ~30 FPS
        except Exception as e:
            print(f"Error updating camera frame: {e}")
            time.sleep(0.1)

def set_board():
    """Initialize robot board and kinematics"""
    if not ROBOT_AVAILABLE:
        return
        
    color_detect.board = board
    color_tracking.board = board
    color_sorting.board = board
    color_palletizing.board = board
    
    color_detect.AK = AK
    color_tracking.AK = AK
    color_sorting.AK = AK
    color_palletizing.AK = AK
    
    color_detect.initMove()
    board.set_buzzer(1900, 0.3, 0.7, 1)

# Utility functions
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def runbymainth(req, pas):
    if callable(req):
        event = threading.Event()
        ret = [event, pas, None]
        QUEUE.put((req, ret))
        count = 0
        
        while ret[2] is None:
            time.sleep(0.01)
            count += 1
            if count > 200:
                break
        if ret[2] is not None:
            if ret[2][0]:
                return ret[2]
            else:
                return (False, __RPC_E03 + " " + ret[2][1])
        else:
            return (False, __RPC_E04)
    else:
        return (False, __RPC_E05)

# ============= RPC FUNCTIONS =============

@_rpc.register
def map(x, in_min, in_max, out_min, out_max):
    return map_value(x, in_min, in_max, out_min, out_max)

@_rpc.register
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    print("SetPWMServo:", args)
    arglen = len(args)
    try:
        servos = args[1:arglen:2]
        pulses = args[2:arglen:2]
        use_times = args[0]
        data = []
        
        dat = zip(servos, pulses)
        for (s, p) in dat:
            pulses = int(map_value(p, 90, -90, 500, 2500))
            data.extend([[s, pulses]])
        board.pwm_servo_set_position(use_times/1000.0, data)
        data.clear()
        
    except Exception as e:
        print('error3:', e)
        ret = (False, __RPC_E03, 'SetPWMServo')
    return ret

@_rpc.register
def SetBusServoPulse(*args, **kwargs):
    ret = (True, (), 'SetBusServoPulse')
    arglen = len(args)
    if (args[1] * 2 + 2) != arglen or arglen < 4:
        return (False, __RPC_E01, 'SetBusServoPulse')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
           if s < 1 or s > 6:
                return (False, __RPC_E02)
        dat = zip(servos, pulses)
        for (s, p) in dat:
            Board.setBusServoPulse(s, p, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoPulse')
    return ret

@_rpc.register
def GetBatteryVoltage():
    ret = (True, 0, 'GetBatteryVoltage')
    if not ROBOT_AVAILABLE:
        # Return demo battery voltage
        return (True, 12.4, 'GetBatteryVoltage')
    try:
        ret = (True, Board.getBattery(), 'GetBatteryVoltage')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBatteryVoltage')
    return ret

@_rpc.register
def LoadFunc(new_func=0):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Loaded function {new_func}", 'LoadFunc')
    return runbymainth(running.loadFunc, (new_func, ))

@_rpc.register
def UnloadFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function unloaded", 'UnloadFunc')
    return runbymainth(running.unloadFunc, ())

@_rpc.register
def StartFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function started", 'StartFunc')
    return runbymainth(running.startFunc, ())

@_rpc.register
def StopFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function stopped", 'StopFunc')
    return runbymainth(running.stopFunc, ())

@_rpc.register
def ColorTracking(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Tracking color {target_color}", 'ColorTracking')
    return runbymainth(color_tracking.setTargetColor, target_color)

@_rpc.register
def ColorSorting(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Sorting color {target_color}", 'ColorSorting')
    return runbymainth(color_sorting.setTargetColor, target_color)

@_rpc.register
def ColorPalletizing(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Palletizing color {target_color}", 'ColorPalletizing')
    return runbymainth(color_palletizing.setTargetColor, target_color)

@_rpc.register
def SetLABValue(*lab_value):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: LAB value set to {lab_value}", 'SetLABValue')
    return runbymainth(lab_adjust.setLABValue, lab_value)

@_rpc.register
def GetLABValue():
    if not ROBOT_AVAILABLE:
        return (True, {"red": ((0, 0, 0), (255, 255, 255))}, 'GetLABValue')
    return (True, lab_adjust.getLABValue()[1], 'GetLABValue')

@_rpc.register
def SaveLABValue(color=''):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: LAB value saved for {color}", 'SaveLABValue')
    return runbymainth(lab_adjust.saveLABValue, (color, ))

@_rpc.register
def HaveLABAdjust():
    return (True, True, 'HaveLABAdjust')

@_rpc.register
def GetRunningFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo Function", 'GetRunningFunc')
    return runbymainth("GetRunningFunc", ())

@_rpc.register
def Heartbeat():
    if not ROBOT_AVAILABLE:
        return (True, "Demo heartbeat", 'Heartbeat')
    return runbymainth(running.doHeartbeat, ())

@_rpc.register
def SetBrushMotor(*args, **kwargs):
    ret = (True, (), 'SetBrushMotor')
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Motor set to {args}", 'SetBrushMotor')
    
    arglen = len(args)
    if 0 != (arglen % 2):
        return (False, __RPC_E01, 'SetBrushMotor')
    try:
        motors = args[0:arglen:2]
        speeds = args[1:arglen:2]
        for m in motors:
            if m < 1 or m > 4:
                return (False, __RPC_E02)
        dat = zip(motors, speeds)

        for m, s in dat:
            Board.setMotor(m, s)
    except:
        ret = (False, __RPC_E03, 'SetBrushMotor')
    return ret

@_rpc.register
def GetSonarDistance():
    if not ROBOT_AVAILABLE:
        return (True, 25.5, 'GetSonarDistance')
    
    global HWSONAR
    ret = (True, 0, 'GetSonarDistance')
    try:
        ret = (True, HWSONAR.getDistance(), 'GetSonarDistance')
    except:
        ret = (False, __RPC_E03, 'GetSonarDistance')
    return ret

# Additional demo/status endpoints
@_rpc.register
def GetSystemInfo():
    return (True, {
        "robot_available": ROBOT_AVAILABLE,
        "camera_active": camera is not None and hasattr(camera, 'opened') and camera.opened,
        "server_time": time.strftime('%Y-%m-%d %H:%M:%S'),
        "demo_mode": not ROBOT_AVAILABLE
    }, 'GetSystemInfo')

# ============= FLASK ROUTES =============

@app.route('/')
def index():
    """Serve the main web interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag"""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    """Get a single snapshot from the camera"""
    global img_show
    if img_show is not None:
        try:
            l_quality = (int(cv2.IMWRITE_JPEG_QUALITY), 100)
            ret, jpg = cv2.imencode('.jpg', img_show, l_quality) 
            jpg_bytes = jpg.tobytes()
            return Response(jpg_bytes, mimetype='image/jpeg')
        except Exception as e:
            print(e)
            return "Error capturing snapshot", 500
    else:
        return "No image available", 404

@app.route('/rpc', methods=['POST'])
def rpc_handler():
    """Handle JSON-RPC requests"""
    try:
        payload = request.get_json()
        if not payload:
            return jsonify({"error": "Empty request body"}), 400
        
        response_obj = _rpc(payload)
        return jsonify(response_obj)
    except Exception as e:
        return jsonify({"error": f"RPC Error: {str(e)}"}), 500

@app.route('/api/robot_status')
def robot_status():
    """Get basic robot status information"""
    try:
        battery = GetBatteryVoltage()
        return jsonify({
            "camera_active": camera is not None and camera.opened,
            "battery_voltage": battery[1] if battery[0] else "N/A"
        })
    except:
        return jsonify({
            "camera_active": False,
            "battery_voltage": "N/A"
        })

def generate_frames():
    """Generate video frames for streaming"""
    global img_show
    while True:
        try:
            if img_show is not None:
                ret, buffer = cv2.imencode('.jpg', img_show, quality)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
        except Exception as e:
            print(f"Error generating frame: {e}")
            time.sleep(0.1)

# ============= HTML TEMPLATE =============

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ArmPi Mini Robot Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #333;
            min-height: 100vh;
        }
        
        .header {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            padding: 1rem 0;
            text-align: center;
            border-bottom: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .header h1 {
            color: white;
            font-size: 2.5rem;
            margin-bottom: 0.5rem;
        }
        
        .status-bar {
            color: rgba(255, 255, 255, 0.8);
            font-size: 1rem;
        }
        
        .container {
            max-width: 1400px;
            margin: 2rem auto;
            padding: 0 1rem;
            display: grid;
            grid-template-columns: 1fr 400px;
            gap: 2rem;
        }
        
        .video-section {
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
            overflow: hidden;
        }
        
        .video-header {
            background: #4a5568;
            color: white;
            padding: 1rem;
            text-align: center;
        }
        
        .video-container {
            position: relative;
            padding: 1rem;
            text-align: center;
        }
        
        #videoStream {
            max-width: 100%;
            height: auto;
            border-radius: 10px;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.1);
        }
        
        .controls-section {
            display: flex;
            flex-direction: column;
            gap: 1rem;
        }
        
        .control-panel {
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
            overflow: hidden;
        }
        
        .panel-header {
            background: #2d3748;
            color: white;
            padding: 1rem;
            text-align: center;
            font-weight: bold;
        }
        
        .panel-content {
            padding: 1.5rem;
        }
        
        .button-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 0.5rem;
            margin-bottom: 1rem;
        }
        
        .btn {
            padding: 0.75rem 1rem;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.3s ease;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.2);
        }
        
        .btn-secondary {
            background: #e2e8f0;
            color: #4a5568;
        }
        
        .btn-secondary:hover {
            background: #cbd5e0;
        }
        
        .btn-danger {
            background: linear-gradient(135deg, #fc466b 0%, #3f5efb 100%);
            color: white;
        }
        
        .color-buttons {
            display: flex;
            gap: 0.5rem;
            flex-wrap: wrap;
            margin-top: 1rem;
        }
        
        .color-btn {
            width: 50px;
            height: 50px;
            border: 3px solid white;
            border-radius: 50%;
            cursor: pointer;
            transition: transform 0.2s ease;
        }
        
        .color-btn:hover {
            transform: scale(1.1);
        }
        
        .color-btn.red { background-color: #e53e3e; }
        .color-btn.green { background-color: #38a169; }
        .color-btn.blue { background-color: #3182ce; }
        .color-btn.yellow { background-color: #d69e2e; }
        .color-btn.purple { background-color: #805ad5; }
        .color-btn.orange { background-color: #dd6b20; }
        
        .servo-control {
            margin-bottom: 1rem;
        }
        
        .servo-control label {
            display: block;
            margin-bottom: 0.5rem;
            font-weight: bold;
            color: #4a5568;
        }
        
        .servo-control input[type="range"] {
            width: 100%;
            margin-bottom: 0.5rem;
        }
        
        .servo-value {
            text-align: center;
            color: #667eea;
            font-weight: bold;
        }
        
        .log-output {
            background: #1a202c;
            color: #green;
            font-family: 'Courier New', monospace;
            padding: 1rem;
            height: 200px;
            overflow-y: auto;
            border-radius: 8px;
            font-size: 0.85rem;
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
                gap: 1rem;
            }
            
            .header h1 {
                font-size: 1.8rem;
            }
            
            .button-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 ArmPi Mini Robot Control</h1>
        <div class="status-bar" id="statusBar">
            Camera: <span id="cameraStatus">Connecting...</span> | 
            Battery: <span id="batteryVoltage">--</span>V
        </div>
    </div>
    
    <div class="container">
        <div class="video-section">
            <div class="video-header">
                <h3>📹 Live Camera Feed</h3>
            </div>
            <div class="video-container">
                <img id="videoStream" src="/video_feed" alt="Robot Camera Feed" 
                     onerror="this.src='data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNjQwIiBoZWlnaHQ9IjQ4MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cmVjdCB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIiBmaWxsPSIjZGRkIi8+PHRleHQgeD0iNTAlIiB5PSI1MCUiIGZvbnQtc2l6ZT0iMTgiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGR5PSIuM2VtIj5DYW1lcmEgTm90IEF2YWlsYWJsZTwvdGV4dD48L3N2Zz4='">
                <div style="margin-top: 1rem;">
                    <button class="btn btn-secondary" onclick="takeSnapshot()">📸 Take Snapshot</button>
                </div>
            </div>
        </div>
        
        <div class="controls-section">
            <div class="control-panel">
                <div class="panel-header">🎮 Robot Functions</div>
                <div class="panel-content">
                    <div class="button-grid">
                        <button class="btn btn-primary" onclick="loadFunction(1)">Color Detection</button>
                        <button class="btn btn-primary" onclick="loadFunction(2)">Color Tracking</button>
                        <button class="btn btn-primary" onclick="loadFunction(3)">Color Sorting</button>
                        <button class="btn btn-primary" onclick="loadFunction(4)">Face Detection</button>
                    </div>
                    <div class="button-grid">
                        <button class="btn btn-secondary" onclick="startFunction()">▶️ Start</button>
                        <button class="btn btn-danger" onclick="stopFunction()">⏹️ Stop</button>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header">🎨 Color Selection</div>
                <div class="panel-content">
                    <div class="color-buttons">
                        <div class="color-btn red" onclick="setTargetColor('red')" title="Red"></div>
                        <div class="color-btn green" onclick="setTargetColor('green')" title="Green"></div>
                        <div class="color-btn blue" onclick="setTargetColor('blue')" title="Blue"></div>
                        <div class="color-btn yellow" onclick="setTargetColor('yellow')" title="Yellow"></div>
                        <div class="color-btn purple" onclick="setTargetColor('purple')" title="Purple"></div>
                        <div class="color-btn orange" onclick="setTargetColor('orange')" title="Orange"></div>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header">🦾 Servo Control</div>
                <div class="panel-content">
                    <div class="servo-control">
                        <label>Servo 1 (Base):</label>
                        <input type="range" id="servo1" min="-90" max="90" value="0" oninput="updateServo(1, this.value)">
                        <div class="servo-value" id="servo1-value">0°</div>
                    </div>
                    <div class="servo-control">
                        <label>Servo 2 (Shoulder):</label>
                        <input type="range" id="servo2" min="-90" max="90" value="0" oninput="updateServo(2, this.value)">
                        <div class="servo-value" id="servo2-value">0°</div>
                    </div>
                    <div class="servo-control">
                        <label>Servo 3 (Elbow):</label>
                        <input type="range" id="servo3" min="-90" max="90" value="0" oninput="updateServo(3, this.value)">
                        <div class="servo-value" id="servo3-value">0°</div>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header">📋 Command Log</div>
                <div class="panel-content">
                    <div class="log-output" id="logOutput">
                        Robot control interface loaded successfully...<br>
                        Ready for commands...<br>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // RPC Helper function
        async function sendRPC(method, params = []) {
            try {
                const response = await fetch('/rpc', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        jsonrpc: "2.0",
                        method: method,
                        params: params,
                        id: Date.now()
                    })
                });
                
                const result = await response.json();
                logMessage(`RPC ${method}: ${JSON.stringify(result)}`);
                return result;
            } catch (error) {
                logMessage(`RPC Error: ${error.message}`);
                console.error('RPC Error:', error);
            }
        }
        
        // Logging function
        function logMessage(message) {
            const logOutput = document.getElementById('logOutput');
            const timestamp = new Date().toLocaleTimeString();
            logOutput.innerHTML += `[${timestamp}] ${message}<br>`;
            logOutput.scrollTop = logOutput.scrollHeight;
        }
        
        // Robot function controls
        async function loadFunction(funcNum) {
            logMessage(`Loading function ${funcNum}...`);
            await sendRPC('LoadFunc', [funcNum]);
        }
        
        async function startFunction() {
            logMessage('Starting current function...');
            await sendRPC('StartFunc');
        }
        
        async function stopFunction() {
            logMessage('Stopping current function...');
            await sendRPC('StopFunc');
        }
        
        // Color tracking
        async function setTargetColor(color) {
            logMessage(`Setting target color to ${color}...`);
            await sendRPC('ColorTracking', [color]);
        }
        
        // Servo control
        async function updateServo(servoNum, angle) {
            document.getElementById(`servo${servoNum}-value`).textContent = `${angle}°`;
            logMessage(`Moving servo ${servoNum} to ${angle}°`);
            await sendRPC('SetPWMServo', [1000, servoNum, parseInt(angle)]);
        }
        
        // Snapshot function
        function takeSnapshot() {
            const link = document.createElement('a');
            link.href = '/snapshot';
            link.download = `robot_snapshot_${new Date().getTime()}.jpg`;
            link.click();
            logMessage('Snapshot captured');
        }
        
        // Status updates
        async function updateStatus() {
            try {
                const response = await fetch('/api/robot_status');
                const status = await response.json();
                
                document.getElementById('cameraStatus').textContent = 
                    status.camera_active ? 'Active' : 'Inactive';
                document.getElementById('batteryVoltage').textContent = 
                    status.battery_voltage !== 'N/A' ? status.battery_voltage.toFixed(1) : '--';
            } catch (error) {
                console.error('Status update error:', error);
            }
        }
        
        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            logMessage('Web interface initialized');
            updateStatus();
            setInterval(updateStatus, 5000); // Update every 5 seconds
        });
        
        // Handle video stream errors
        document.getElementById('videoStream').addEventListener('error', function() {
            logMessage('Video stream connection lost');
        });
        
        document.getElementById('videoStream').addEventListener('load', function() {
            logMessage('Video stream connected');
        });
    </script>
</body>
</html>
'''

if __name__ == '__main__':
    print("Starting ArmPi Mini Web Server...")
    
    # Initialize robot hardware
    initialize_robot()
    
    # Disable werkzeug logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    # Run the Flask application
    print("Web server running on http://0.0.0.0:8000")
    print("Access the robot control interface in your browser")
    app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)