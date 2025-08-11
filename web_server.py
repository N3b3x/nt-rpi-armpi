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

# Custom RPC registry for demo mode support
rpc_functions = {}

def rpc_register(func):
    """Custom decorator to register RPC functions"""
    rpc_functions[func.__name__] = func
    # Add to JsonRpc methods dictionary
    _rpc.methods[func.__name__] = func
    return func

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

def sync_with_mjpg_server():
    """Sync img_show with mjpg_server for shared display"""
    global img_show
    while True:
        try:
            import mjpg_server
            if hasattr(mjpg_server, 'img_show') and mjpg_server.img_show is not None:
                img_show = mjpg_server.img_show.copy()
            time.sleep(0.033)  # ~30 FPS
        except Exception as e:
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

@rpc_register
def map(x, in_min, in_max, out_min, out_max):
    return map_value(x, in_min, in_max, out_min, out_max)

@rpc_register
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    print("SetPWMServo:", args)
    
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Servo moved with params {args}", 'SetPWMServo')
    
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

@rpc_register
def SetBusServoPulse(*args, **kwargs):
    ret = (True, (), 'SetBusServoPulse')
    
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Bus servo pulse set with params {args}", 'SetBusServoPulse')
    
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

@rpc_register
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

@rpc_register
def LoadFunc(new_func=0):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Loaded function {new_func}", 'LoadFunc')
    return runbymainth(running.loadFunc, (new_func, ))

@rpc_register
def UnloadFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function unloaded", 'UnloadFunc')
    return runbymainth(running.unloadFunc, ())

@rpc_register
def StartFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function started", 'StartFunc')
    return runbymainth(running.startFunc, ())

@rpc_register
def StopFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo: Function stopped", 'StopFunc')
    return runbymainth(running.stopFunc, ())

@rpc_register
def ColorTracking(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Tracking color {target_color}", 'ColorTracking')
    return runbymainth(color_tracking.setTargetColor, target_color)

@rpc_register
def ColorSorting(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Sorting color {target_color}", 'ColorSorting')
    return runbymainth(color_sorting.setTargetColor, target_color)

@rpc_register
def ColorPalletizing(*target_color):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Palletizing color {target_color}", 'ColorPalletizing')
    return runbymainth(color_palletizing.setTargetColor, target_color)

@rpc_register
def SetLABValue(*lab_value):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: LAB value set to {lab_value}", 'SetLABValue')
    return runbymainth(lab_adjust.setLABValue, lab_value)

@rpc_register
def GetLABValue():
    if not ROBOT_AVAILABLE:
        return (True, {"red": ((0, 0, 0), (255, 255, 255))}, 'GetLABValue')
    return (True, lab_adjust.getLABValue()[1], 'GetLABValue')

@rpc_register
def SaveLABValue(color=''):
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: LAB value saved for {color}", 'SaveLABValue')
    return runbymainth(lab_adjust.saveLABValue, (color, ))

@rpc_register
def HaveLABAdjust():
    return (True, True, 'HaveLABAdjust')

@rpc_register
def GetRunningFunc():
    if not ROBOT_AVAILABLE:
        return (True, "Demo Function", 'GetRunningFunc')
    return runbymainth("GetRunningFunc", ())

@rpc_register
def Heartbeat():
    if not ROBOT_AVAILABLE:
        return (True, "Demo heartbeat", 'Heartbeat')
    return runbymainth(running.doHeartbeat, ())

@rpc_register
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

@rpc_register
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
@rpc_register
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
    """Handle JSON-RPC requests with custom registry and demo mode support"""
    try:
        payload = request.get_json()
        if not payload:
            return jsonify({"error": "Empty request body"}), 400
        
        # Handle JSON-RPC 2.0 format
        if isinstance(payload, dict):
            method_name = payload.get('method')
            params = payload.get('params', [])
            rpc_id = payload.get('id')
            
            if method_name in rpc_functions:
                try:
                    # Call the function with parameters
                    if isinstance(params, list):
                        result = rpc_functions[method_name](*params)
                    elif isinstance(params, dict):
                        result = rpc_functions[method_name](**params)
                    else:
                        result = rpc_functions[method_name]()
                    
                    # Return JSON-RPC 2.0 response format
                    return jsonify({
                        "jsonrpc": "2.0",
                        "result": result,
                        "id": rpc_id
                    })
                except Exception as e:
                    return jsonify({
                        "jsonrpc": "2.0",
                        "error": {
                            "code": -32603,
                            "message": f"Internal error: {str(e)}"
                        },
                        "id": rpc_id
                    })
            else:
                return jsonify({
                    "jsonrpc": "2.0",
                    "error": {
                        "code": -32601,
                        "message": f"Method not found: {method_name}"
                    },
                    "id": rpc_id
                })
        else:
            # Fallback to original RPC handler for compatibility
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
    <title>🤖 ArmPi Mini Robot Control</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" rel="stylesheet">
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        :root {
            --primary-gradient: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            --secondary-gradient: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            --accent-gradient: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
            --success-gradient: linear-gradient(135deg, #43e97b 0%, #38f9d7 100%);
            --warning-gradient: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
            --dark-gradient: linear-gradient(135deg, #2c3e50 0%, #34495e 100%);
            --glass-bg: rgba(255, 255, 255, 0.1);
            --glass-border: rgba(255, 255, 255, 0.2);
            --shadow-color: rgba(0, 0, 0, 0.2);
            --text-light: rgba(255, 255, 255, 0.9);
            --text-secondary: rgba(255, 255, 255, 0.7);
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: var(--primary-gradient);
            color: #333;
            min-height: 100vh;
            overflow-x: hidden;
            position: relative;
        }
        
        /* Animated background particles */
        body::before {
            content: '';
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: url('data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><circle cx="20" cy="20" r="2" fill="rgba(255,255,255,0.1)"><animate attributeName="cy" values="20;80;20" dur="3s" repeatCount="indefinite"/></circle><circle cx="40" cy="40" r="1" fill="rgba(255,255,255,0.05)"><animate attributeName="cy" values="40;10;40" dur="2s" repeatCount="indefinite"/></circle><circle cx="60" cy="60" r="1.5" fill="rgba(255,255,255,0.08)"><animate attributeName="cy" values="60;90;60" dur="4s" repeatCount="indefinite"/></circle><circle cx="80" cy="30" r="1" fill="rgba(255,255,255,0.06)"><animate attributeName="cy" values="30;70;30" dur="2.5s" repeatCount="indefinite"/></circle></svg>') repeat;
            animation: float 20s infinite linear;
            pointer-events: none;
            z-index: -1;
        }
        
        @keyframes float {
            0% { transform: translateY(0px) rotate(0deg); }
            100% { transform: translateY(-100vh) rotate(360deg); }
        }
        
        .header {
            background: var(--glass-bg);
            backdrop-filter: blur(20px);
            padding: 2rem 0;
            text-align: center;
            border-bottom: 1px solid var(--glass-border);
            position: relative;
            overflow: hidden;
            animation: slideDown 1s ease-out;
        }
        
        .header::before {
            content: '';
            position: absolute;
            top: -50%;
            left: -50%;
            width: 200%;
            height: 200%;
            background: radial-gradient(circle, rgba(255,255,255,0.1) 0%, transparent 70%);
            animation: rotate 20s linear infinite;
        }
        
        .header h1 {
            color: white;
            font-size: 3rem;
            margin-bottom: 0.5rem;
            font-weight: 700;
            text-shadow: 0 4px 8px var(--shadow-color);
            animation: glow 2s ease-in-out infinite alternate;
            position: relative;
            z-index: 2;
        }
        
        .header h1::before {
            content: '🤖';
            display: inline-block;
            margin-right: 0.5rem;
            animation: bounce 2s infinite;
        }
        
        .status-bar {
            color: var(--text-secondary);
            font-size: 1.1rem;
            position: relative;
            z-index: 2;
            animation: fadeIn 1.5s ease-in;
        }
        
        .status-indicator {
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 5px;
            animation: pulse 2s infinite;
        }
        
        .status-indicator.active {
            background: #43e97b;
            box-shadow: 0 0 10px #43e97b;
        }
        
        .status-indicator.inactive {
            background: #e74c3c;
            box-shadow: 0 0 10px #e74c3c;
        }
        
        @keyframes slideDown {
            from { transform: translateY(-100%); opacity: 0; }
            to { transform: translateY(0); opacity: 1; }
        }
        
        @keyframes glow {
            from { text-shadow: 0 4px 8px var(--shadow-color), 0 0 20px rgba(255,255,255,0.2); }
            to { text-shadow: 0 4px 8px var(--shadow-color), 0 0 30px rgba(255,255,255,0.4); }
        }
        
        @keyframes bounce {
            0%, 20%, 50%, 80%, 100% { transform: translateY(0); }
            40% { transform: translateY(-10px); }
            60% { transform: translateY(-5px); }
        }
        
        @keyframes pulse {
            0% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.7; transform: scale(1.1); }
            100% { opacity: 1; transform: scale(1); }
        }
        
        @keyframes rotate {
            from { transform: rotate(0deg); }
            to { transform: rotate(360deg); }
        }
        
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(20px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        .container {
            max-width: 1400px;
            margin: 2rem auto;
            padding: 0 1rem;
            display: grid;
            grid-template-columns: 1fr 420px;
            gap: 2rem;
            animation: fadeInUp 1s ease-out 0.5s both;
        }
        
        .video-section {
            background: var(--glass-bg);
            backdrop-filter: blur(20px);
            border: 1px solid var(--glass-border);
            border-radius: 20px;
            box-shadow: 0 20px 40px var(--shadow-color);
            overflow: hidden;
            transition: all 0.3s ease;
            animation: slideInLeft 1s ease-out;
        }
        
        .video-section:hover {
            transform: translateY(-5px);
            box-shadow: 0 25px 50px rgba(0, 0, 0, 0.3);
        }
        
        .video-header {
            background: var(--dark-gradient);
            color: white;
            padding: 1.5rem;
            text-align: center;
            position: relative;
            overflow: hidden;
        }
        
        .video-header::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
            animation: shine 3s infinite;
        }
        
        .video-header h3 {
            font-size: 1.3rem;
            font-weight: 600;
            margin: 0;
            position: relative;
            z-index: 2;
        }
        
        .video-container {
            position: relative;
            padding: 2rem;
            text-align: center;
            background: rgba(255, 255, 255, 0.05);
        }
        
        #videoStream {
            max-width: 100%;
            height: auto;
            border-radius: 15px;
            box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
            border: 2px solid rgba(255, 255, 255, 0.1);
            transition: all 0.3s ease;
        }
        
        #videoStream:hover {
            transform: scale(1.02);
            box-shadow: 0 15px 35px rgba(0, 0, 0, 0.3);
        }
        
        .controls-section {
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
            animation: slideInRight 1s ease-out;
        }
        
        .control-panel {
            background: var(--glass-bg);
            backdrop-filter: blur(20px);
            border: 1px solid var(--glass-border);
            border-radius: 20px;
            box-shadow: 0 15px 35px var(--shadow-color);
            overflow: hidden;
            transition: all 0.3s ease;
            animation: fadeInScale 0.6s ease-out;
        }
        
        .control-panel:hover {
            transform: translateY(-3px);
            box-shadow: 0 20px 45px rgba(0, 0, 0, 0.25);
        }
        
        .panel-header {
            background: var(--dark-gradient);
            color: white;
            padding: 1.2rem;
            text-align: center;
            font-weight: 600;
            font-size: 1.1rem;
            position: relative;
            overflow: hidden;
        }
        
        .panel-header::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
            animation: shine 4s infinite;
        }
        
        .panel-content {
            padding: 2rem;
            background: rgba(255, 255, 255, 0.05);
        }
        
        @keyframes fadeInUp {
            from { opacity: 0; transform: translateY(30px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        @keyframes slideInLeft {
            from { opacity: 0; transform: translateX(-50px); }
            to { opacity: 1; transform: translateX(0); }
        }
        
        @keyframes slideInRight {
            from { opacity: 0; transform: translateX(50px); }
            to { opacity: 1; transform: translateX(0); }
        }
        
        @keyframes fadeInScale {
            from { opacity: 0; transform: scale(0.9); }
            to { opacity: 1; transform: scale(1); }
        }
        
        @keyframes shine {
            0% { left: -100%; }
            100% { left: 100%; }
        }
        
        .button-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 0.5rem;
            margin-bottom: 1rem;
        }
        
        .btn {
            padding: 1rem 1.5rem;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.9rem;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            position: relative;
            overflow: hidden;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }
        
        .btn:hover::before {
            left: 100%;
        }
        
        .btn-primary {
            background: var(--accent-gradient);
            color: white;
            box-shadow: 0 4px 15px rgba(79, 172, 254, 0.3);
        }
        
        .btn-primary:hover {
            transform: translateY(-3px) scale(1.02);
            box-shadow: 0 8px 25px rgba(79, 172, 254, 0.4);
        }
        
        .btn-secondary {
            background: var(--glass-bg);
            color: white;
            box-shadow: 0 4px 15px var(--shadow-color);
        }
        
        .btn-secondary:hover {
            background: rgba(255, 255, 255, 0.2);
            transform: translateY(-3px) scale(1.02);
            box-shadow: 0 8px 25px var(--shadow-color);
        }
        
        .btn-danger {
            background: var(--secondary-gradient);
            color: white;
            box-shadow: 0 4px 15px rgba(240, 147, 251, 0.3);
        }
        
        .btn-danger:hover {
            transform: translateY(-3px) scale(1.02);
            box-shadow: 0 8px 25px rgba(240, 147, 251, 0.4);
        }
        
        .btn-success {
            background: var(--success-gradient);
            color: white;
            box-shadow: 0 4px 15px rgba(67, 233, 123, 0.3);
        }
        
        .btn-success:hover {
            transform: translateY(-3px) scale(1.02);
            box-shadow: 0 8px 25px rgba(67, 233, 123, 0.4);
        }
        
        .color-buttons {
            display: flex;
            gap: 0.8rem;
            flex-wrap: wrap;
            margin-top: 1.5rem;
            justify-content: center;
        }
        
        .color-btn {
            width: 60px;
            height: 60px;
            border: 3px solid rgba(255, 255, 255, 0.3);
            border-radius: 50%;
            cursor: pointer;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            position: relative;
            overflow: hidden;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.2);
        }
        
        .color-btn::before {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 0;
            height: 0;
            background: rgba(255, 255, 255, 0.3);
            border-radius: 50%;
            transition: all 0.3s ease;
            transform: translate(-50%, -50%);
        }
        
        .color-btn:hover {
            transform: scale(1.15) rotate(5deg);
            border-color: rgba(255, 255, 255, 0.8);
            box-shadow: 0 10px 25px rgba(0, 0, 0, 0.3);
        }
        
        .color-btn:hover::before {
            width: 100%;
            height: 100%;
        }
        
        .color-btn.red { 
            background: linear-gradient(135deg, #e53e3e 0%, #ff6b6b 100%);
            box-shadow: 0 5px 15px rgba(229, 62, 62, 0.4);
        }
        .color-btn.green { 
            background: linear-gradient(135deg, #38a169 0%, #48bb78 100%);
            box-shadow: 0 5px 15px rgba(56, 161, 105, 0.4);
        }
        .color-btn.blue { 
            background: linear-gradient(135deg, #3182ce 0%, #4299e1 100%);
            box-shadow: 0 5px 15px rgba(49, 130, 206, 0.4);
        }
        .color-btn.yellow { 
            background: linear-gradient(135deg, #d69e2e 0%, #ecc94b 100%);
            box-shadow: 0 5px 15px rgba(214, 158, 46, 0.4);
        }
        .color-btn.purple { 
            background: linear-gradient(135deg, #805ad5 0%, #9f7aea 100%);
            box-shadow: 0 5px 15px rgba(128, 90, 213, 0.4);
        }
        .color-btn.orange { 
            background: linear-gradient(135deg, #dd6b20 0%, #ed8936 100%);
            box-shadow: 0 5px 15px rgba(221, 107, 32, 0.4);
        }
        
        .servo-control {
            margin-bottom: 1.5rem;
            padding: 1rem;
            background: rgba(255, 255, 255, 0.05);
            border-radius: 15px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            transition: all 0.3s ease;
        }
        
        .servo-control:hover {
            background: rgba(255, 255, 255, 0.08);
            transform: translateY(-2px);
        }
        
        .servo-control label {
            display: block;
            margin-bottom: 0.8rem;
            font-weight: 600;
            color: var(--text-light);
            font-size: 1rem;
            text-shadow: 0 2px 4px var(--shadow-color);
        }
        
        .servo-control input[type="range"] {
            width: 100%;
            margin-bottom: 0.8rem;
            height: 8px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.2);
            outline: none;
            -webkit-appearance: none;
            appearance: none;
            transition: all 0.3s ease;
        }
        
        .servo-control input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--accent-gradient);
            cursor: pointer;
            box-shadow: 0 4px 12px rgba(79, 172, 254, 0.4);
            transition: all 0.3s ease;
        }
        
        .servo-control input[type="range"]::-webkit-slider-thumb:hover {
            transform: scale(1.2);
            box-shadow: 0 6px 16px rgba(79, 172, 254, 0.6);
        }
        
        .servo-control input[type="range"]::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--accent-gradient);
            cursor: pointer;
            border: none;
            box-shadow: 0 4px 12px rgba(79, 172, 254, 0.4);
        }
        
        .servo-value {
            text-align: center;
            color: var(--text-light);
            font-weight: 600;
            font-size: 1.1rem;
            padding: 0.5rem;
            background: var(--accent-gradient);
            border-radius: 10px;
            text-shadow: 0 2px 4px var(--shadow-color);
            box-shadow: 0 4px 12px rgba(79, 172, 254, 0.3);
        }
        
        .log-output {
            background: linear-gradient(135deg, #1a202c 0%, #2d3748 100%);
            color: #00ff88;
            font-family: 'JetBrains Mono', 'Courier New', monospace;
            padding: 1.5rem;
            height: 250px;
            overflow-y: auto;
            border-radius: 15px;
            font-size: 0.9rem;
            line-height: 1.4;
            border: 1px solid rgba(0, 255, 136, 0.2);
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
            position: relative;
        }
        
        .log-output::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 1px;
            background: linear-gradient(90deg, transparent, #00ff88, transparent);
            animation: scanline 2s linear infinite;
        }
        
        .log-output::-webkit-scrollbar {
            width: 8px;
        }
        
        .log-output::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 4px;
        }
        
        .log-output::-webkit-scrollbar-thumb {
            background: var(--accent-gradient);
            border-radius: 4px;
        }
        
        .log-output::-webkit-scrollbar-thumb:hover {
            background: var(--success-gradient);
        }
        
        @keyframes scanline {
            0% { opacity: 1; }
            50% { opacity: 0.3; }
            100% { opacity: 1; }
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
        <h1>ArmPi Mini Robot Control</h1>
        <div class="status-bar" id="statusBar">
            <span class="status-indicator active" id="cameraIndicator"></span>
            Camera: <span id="cameraStatus">Connecting...</span> 
            <span style="margin: 0 1rem;">|</span>
            <span class="status-indicator active" id="batteryIndicator"></span>
            Battery: <span id="batteryVoltage">--</span>V
            <span style="margin: 0 1rem;">|</span>
            <i class="fas fa-clock"></i> <span id="currentTime">--:--:--</span>
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
                <div class="panel-header"><i class="fas fa-gamepad"></i> Robot Functions</div>
                <div class="panel-content">
                    <div class="button-grid">
                        <button class="btn btn-primary" onclick="loadFunction(1)">
                            <i class="fas fa-eye"></i> Color Detection
                        </button>
                        <button class="btn btn-primary" onclick="loadFunction(2)">
                            <i class="fas fa-crosshairs"></i> Color Tracking
                        </button>
                        <button class="btn btn-primary" onclick="loadFunction(3)">
                            <i class="fas fa-sort"></i> Color Sorting
                        </button>
                        <button class="btn btn-primary" onclick="loadFunction(4)">
                            <i class="fas fa-user"></i> Face Detection
                        </button>
                    </div>
                    <div class="button-grid" style="margin-top: 1rem;">
                        <button class="btn btn-success" onclick="startFunction()">
                            <i class="fas fa-play"></i> Start
                        </button>
                        <button class="btn btn-danger" onclick="stopFunction()">
                            <i class="fas fa-stop"></i> Stop
                        </button>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header"><i class="fas fa-palette"></i> Color Selection</div>
                <div class="panel-content">
                    <div class="color-buttons">
                        <div class="color-btn red" onclick="setTargetColor('red')" title="Red">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                        <div class="color-btn green" onclick="setTargetColor('green')" title="Green">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                        <div class="color-btn blue" onclick="setTargetColor('blue')" title="Blue">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                        <div class="color-btn yellow" onclick="setTargetColor('yellow')" title="Yellow">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                        <div class="color-btn purple" onclick="setTargetColor('purple')" title="Purple">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                        <div class="color-btn orange" onclick="setTargetColor('orange')" title="Orange">
                            <i class="fas fa-circle" style="color: rgba(255,255,255,0.8); font-size: 1.5rem;"></i>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header"><i class="fas fa-robot"></i> Servo Control</div>
                <div class="panel-content">
                    <div class="servo-control">
                        <label><i class="fas fa-compass"></i> Servo 1 (Base):</label>
                        <input type="range" id="servo1" min="-90" max="90" value="0" oninput="updateServo(1, this.value)">
                        <div class="servo-value" id="servo1-value">0°</div>
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-arrows-alt-v"></i> Servo 2 (Shoulder):</label>
                        <input type="range" id="servo2" min="-90" max="90" value="0" oninput="updateServo(2, this.value)">
                        <div class="servo-value" id="servo2-value">0°</div>
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-angle-double-right"></i> Servo 3 (Elbow):</label>
                        <input type="range" id="servo3" min="-90" max="90" value="0" oninput="updateServo(3, this.value)">
                        <div class="servo-value" id="servo3-value">0°</div>
                    </div>
                </div>
            </div>
            
            <div class="control-panel">
                <div class="panel-header"><i class="fas fa-terminal"></i> Command Log</div>
                <div class="panel-content">
                    <div class="log-output" id="logOutput">
                        <span style="color: #00ff88;">[SYSTEM]</span> Robot control interface loaded successfully...<br>
                        <span style="color: #4facfe;">[READY]</span> Waiting for commands...<br>
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
                
                if (result.error) {
                    logMessage(`❌ RPC ${method} failed: ${result.error.message}`, 'error');
                } else {
                    logMessage(`🔄 RPC ${method} executed`, 'rpc');
                }
                
                return result;
            } catch (error) {
                logMessage(`❌ RPC Connection Error: ${error.message}`, 'error');
                console.error('RPC Error:', error);
                return null;
            }
        }
        
        // Logging function
        function logMessage(message, type = 'info') {
            const logOutput = document.getElementById('logOutput');
            const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
            
            let color = '#00ff88'; // default green
            let prefix = '[INFO]';
            
            switch(type) {
                case 'success':
                    color = '#43e97b';
                    prefix = '[SUCCESS]';
                    break;
                case 'error':
                    color = '#fc466b';
                    prefix = '[ERROR]';
                    break;
                case 'warning':
                    color = '#fee140';
                    prefix = '[WARNING]';
                    break;
                case 'system':
                    color = '#4facfe';
                    prefix = '[SYSTEM]';
                    break;
                case 'rpc':
                    color = '#9f7aea';
                    prefix = '[RPC]';
                    break;
            }
            
            logOutput.innerHTML += `<span style="color: #666;">[${timestamp}]</span> <span style="color: ${color};">${prefix}</span> ${message}<br>`;
            logOutput.scrollTop = logOutput.scrollHeight;
            
            // Limit log lines to prevent memory issues
            const lines = logOutput.innerHTML.split('<br>');
            if (lines.length > 100) {
                logOutput.innerHTML = lines.slice(-50).join('<br>');
            }
        }
        
        // Robot function controls
        async function loadFunction(funcNum) {
            const functionNames = {
                1: 'Color Detection',
                2: 'Color Tracking', 
                3: 'Color Sorting',
                4: 'Face Detection'
            };
            logMessage(`Loading ${functionNames[funcNum]}...`, 'system');
            const result = await sendRPC('LoadFunc', [funcNum]);
            if (result && result.result) {
                logMessage(`✅ ${functionNames[funcNum]} loaded successfully`, 'success');
            }
        }
        
        async function startFunction() {
            logMessage('▶️ Starting current function...', 'system');
            const result = await sendRPC('StartFunc');
            if (result && result.result) {
                logMessage('✅ Function started successfully', 'success');
            }
        }
        
        async function stopFunction() {
            logMessage('⏹️ Stopping current function...', 'system');
            const result = await sendRPC('StopFunc');
            if (result && result.result) {
                logMessage('✅ Function stopped successfully', 'success');
            }
        }
        
        // Color tracking
        async function setTargetColor(color) {
            logMessage(`🎨 Setting target color to ${color}...`, 'system');
            const result = await sendRPC('ColorTracking', [color]);
            if (result && result.result) {
                logMessage(`✅ Target color set to ${color}`, 'success');
            }
        }
        
        // Servo control
        async function updateServo(servoNum, angle) {
            document.getElementById(`servo${servoNum}-value`).textContent = `${angle}°`;
            logMessage(`🦾 Moving servo ${servoNum} to ${angle}°`, 'system');
            const result = await sendRPC('SetPWMServo', [1000, servoNum, parseInt(angle)]);
            if (result && result.result) {
                logMessage(`✅ Servo ${servoNum} moved to ${angle}°`, 'success');
            }
        }
        
        // Snapshot function
        function takeSnapshot() {
            const link = document.createElement('a');
            link.href = '/snapshot';
            link.download = `robot_snapshot_${new Date().getTime()}.jpg`;
            link.click();
            logMessage('📸 Snapshot captured and downloaded', 'success');
        }
        
        // Status updates
        async function updateStatus() {
            try {
                const response = await fetch('/api/robot_status');
                const status = await response.json();
                
                // Update camera status
                const cameraStatus = document.getElementById('cameraStatus');
                const cameraIndicator = document.getElementById('cameraIndicator');
                if (status.camera_active) {
                    cameraStatus.textContent = 'Active';
                    cameraIndicator.className = 'status-indicator active';
                } else {
                    cameraStatus.textContent = 'Inactive';
                    cameraIndicator.className = 'status-indicator inactive';
                }
                
                // Update battery status
                const batteryVoltage = document.getElementById('batteryVoltage');
                const batteryIndicator = document.getElementById('batteryIndicator');
                if (status.battery_voltage !== 'N/A') {
                    const voltage = parseFloat(status.battery_voltage);
                    batteryVoltage.textContent = voltage.toFixed(1);
                    batteryIndicator.className = voltage > 11.0 ? 'status-indicator active' : 'status-indicator inactive';
                } else {
                    batteryVoltage.textContent = '--';
                    batteryIndicator.className = 'status-indicator inactive';
                }
            } catch (error) {
                console.error('Status update error:', error);
                document.getElementById('cameraIndicator').className = 'status-indicator inactive';
                document.getElementById('batteryIndicator').className = 'status-indicator inactive';
            }
        }
        
        // Update current time
        function updateTime() {
            const now = new Date();
            const timeString = now.toLocaleTimeString('en-US', { 
                hour12: false,
                hour: '2-digit',
                minute: '2-digit',
                second: '2-digit'
            });
            document.getElementById('currentTime').textContent = timeString;
        }
        
        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            logMessage('🚀 Web interface initialized successfully!');
            updateStatus();
            updateTime();
            setInterval(updateStatus, 5000); // Update every 5 seconds
            setInterval(updateTime, 1000); // Update every second
            
            // Add loading animation to buttons
            document.querySelectorAll('.btn').forEach(btn => {
                btn.addEventListener('click', function() {
                    this.style.transform = 'scale(0.95)';
                    setTimeout(() => {
                        this.style.transform = '';
                    }, 150);
                });
            });
        });
        
        // Handle video stream errors
        document.getElementById('videoStream').addEventListener('error', function() {
            logMessage('📺 Video stream connection lost', 'warning');
        });
        
        document.getElementById('videoStream').addEventListener('load', function() {
            logMessage('📺 Video stream connected successfully', 'success');
        });
        
        // Add welcome message with delay
        setTimeout(() => {
            logMessage('🤖 Welcome to ArmPi Mini Robot Control Interface!', 'system');
            logMessage('💡 Click buttons to control the robot', 'info');
            logMessage('🎮 Use sliders to move servos in real-time', 'info');
        }, 1000);
    </script>
</body>
</html>
'''

def startWebServer(camera_instance=None, board_instance=None, ak_instance=None, queue_instance=None, robot_available=True):
    """
    Start the web server with given instances from main ArmPi_mini.py
    """
    global camera, board, AK, QUEUE, img_show, ROBOT_AVAILABLE
    
    # Set robot availability flag
    ROBOT_AVAILABLE = robot_available
    
    # Use provided instances from main program
    if camera_instance:
        camera = camera_instance
    if board_instance:
        board = board_instance
    if ak_instance:
        AK = ak_instance
    if queue_instance:
        QUEUE = queue_instance
    
    # Set up robot modules if available
    if ROBOT_AVAILABLE and board and AK:
        try:
            set_board()
        except Exception as e:
            print(f"Warning: Could not initialize robot board in web server: {e}")
    
    # Start camera frame sync thread to share with mjpg_server
    if camera_instance:
        # Use shared mjpg_server frames for consistency
        threading.Thread(target=sync_with_mjpg_server, daemon=True).start()
    elif ROBOT_AVAILABLE:
        initialize_robot()
    else:
        # Fallback to demo mode
        threading.Thread(target=create_demo_frames, daemon=True).start()
    
    # Disable werkzeug logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    print("🌐 Web Server starting on http://0.0.0.0:8000")
    print("   Access the robot control interface in your browser")
    
    # Run the Flask application
    app.run(host='0.0.0.0', port=8000, debug=False, threaded=True, use_reloader=False)

if __name__ == '__main__':
    print("Starting ArmPi Mini Web Server in standalone mode...")
    
    # Set ROBOT_AVAILABLE to False when running standalone since we don't have hardware
    ROBOT_AVAILABLE = False
    print(f"Robot modules available: {ROBOT_AVAILABLE}")
    
    # Initialize robot hardware
    initialize_robot()
    
    # Disable werkzeug logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    # Run the Flask application
    print("Web server running on http://0.0.0.0:8000")
    print("Access the robot control interface in your browser")
    app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)