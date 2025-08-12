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
board = None

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

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def safe_board_call(method_name, *args, **kwargs):
    """Safely call a board method whether it's on the instance or class."""
    try:
        if 'board' in globals() and board is not None and hasattr(board, method_name):
            return getattr(board, method_name)(*args, **kwargs)
        # Fallback to class/static style if available
        if 'Board' in globals() and hasattr(Board, method_name):
            return getattr(Board, method_name)(*args, **kwargs)
    except Exception as e:
        print(f"Board call error {method_name}: {e}")
        raise
    raise NameError("Board is not available")

def load_servo_map():
    try:
        import yaml
        map_path = os.path.join(current_dir, 'config', 'servo_map.yaml')
        if not os.path.exists(map_path):
            return None
        with open(map_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to load servo_map.yaml: {e}")
        return None

SERVO_MAP = load_servo_map()

def apply_servo_map(servo_id, angle_deg_or_pulse):
    """Map angle (based on YAML degrees_min/max if present) or raw pulse to final pulse using config (invert/offset/range)."""
    value = angle_deg_or_pulse
    # Detect raw microseconds
    if isinstance(value, (int, float)) and value >= 300:
        pulse = int(value)
    else:
        # Treat as degrees. Prefer YAML degrees_min/max if configured.
        deg_min_cfg, deg_max_cfg = -90, 180
        pmin_cfg, pmax_cfg = 500, 2500
        off_cfg, inv_cfg = 0, False
        if SERVO_MAP and 'servos' in SERVO_MAP:
            for s in SERVO_MAP['servos']:
                if int(s.get('id', -1)) == int(servo_id):
                    deg_min_cfg = int(s.get('degrees_min', 0))
                    deg_max_cfg = int(s.get('degrees_max', 180))
                    pmin_cfg = int(s.get('pulse_min', 500))
                    pmax_cfg = int(s.get('pulse_max', 2500))
                    off_cfg = int(s.get('offset_pulse', 0))
                    inv_cfg = bool(s.get('invert', False))
                    break
        try:
            angle = float(value)
        except Exception:
            angle = 0.0
        # Map angle from YAML range to pulse range
        pulse = int(map_value(angle, deg_min_cfg, deg_max_cfg, pmin_cfg, pmax_cfg))
        # Apply invert around mid if requested
        if inv_cfg:
            mid = (pmin_cfg + pmax_cfg) // 2
            pulse = mid - (pulse - mid)
        # Apply offset and clamp
        pulse = clamp(pulse + off_cfg, pmin_cfg, pmax_cfg)

    return clamp(pulse, 500, 2500)

def read_battery_voltage_safe():
    try:
        if not ROBOT_AVAILABLE:
            return 12.4
        # Try instance then class
        if 'board' in globals() and board is not None:
            if hasattr(board, 'getBattery'):
                return board.getBattery()
            if hasattr(board, 'get_battery'):
                return board.get_battery()
        if 'Board' in globals():
            if hasattr(Board, 'getBattery'):
                return Board.getBattery()
        return None
    except Exception as e:
        print(f"read_battery_voltage_safe error: {e}")
        return None

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
        servos = list(args[1:arglen:2])
        angles_or_pulses = list(args[2:arglen:2])
        use_times_ms = args[0]
        data = []

        for (servo_id, value) in zip(servos, angles_or_pulses):
            pulse_us = apply_servo_map(servo_id, value)
            data.append([servo_id, pulse_us])

        # Call through board instance safely
        safe_board_call('pwm_servo_set_position', use_times_ms/1000.0, data)
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
        # Batch call for bus servos via SDK API
        positions = []
        for (s, p) in zip(servos, pulses):
            positions.append([int(s), int(clamp(p, 500, 2500))])
        safe_board_call('bus_servo_set_position', use_times/1000.0, positions)
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
        vb = read_battery_voltage_safe()
        if vb is None:
            return (False, __RPC_E03, 'GetBatteryVoltage')
        ret = (True, vb, 'GetBatteryVoltage')
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
        # Batch speeds into one call according to SDK signature
        speeds_list = []
        for m, s in zip(motors, speeds):
            speeds_list.append([int(m), float(s)])
        # Prefer set_motor_speed; fall back to set_motor_duty if needed
        try:
            safe_board_call('set_motor_speed', speeds_list)
        except Exception:
            safe_board_call('set_motor_duty', speeds_list)
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

@app.route('/api/servo_config')
def servo_config():
    cfg = SERVO_MAP if SERVO_MAP else {"servos": []}
    # Provide sensible defaults and ensure ids are present
    out = {"servos": []}
    seen = set()
    for s in cfg.get('servos', []):
        sid = int(s.get('id', -1))
        if sid <= 0 or sid in seen:
            continue
        seen.add(sid)
        out['servos'].append({
            "id": sid,
            "degrees_min": int(s.get('degrees_min', 0)),
            "degrees_max": int(s.get('degrees_max', 180)),
            "default_degrees": int(s.get('default_degrees', (int(s.get('degrees_min', 0)) + int(s.get('degrees_max', 180)))//2))
        })
    return jsonify(out)

@app.route('/3d')
def landing_3d():
    """3D Interactive Landing Page"""
    return render_template_string(HTML_3D_TEMPLATE)

@app.route('/models/<filename>')
def serve_3d_model(filename):
    """Serve 3D model files"""
    try:
        return send_from_directory('CAD', filename)
    except Exception as e:
        return f"Model not found: {e}", 404

@app.route('/combined')
def combined_view():
    """Combined Camera + 3D page (stacked)."""
    return render_template_string(HTML_COMBINED_TEMPLATE)

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
            text-align: left;
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
                    <a href="/3d" class="btn btn-primary" style="display: inline-block; text-decoration: none; margin-left: 0.5rem;"><i class="fas fa-cube"></i> 3D Model Viewer</a>
                    <a href="/combined" class="btn btn-secondary" style="display: inline-block; text-decoration: none; margin-left: 0.5rem;"><i class="fas fa-columns"></i> Combined View</a>
                </div>
                <!-- Big command log under the camera -->
                <div class="control-panel" style="margin-top: 1rem;">
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
                <div class="panel-header"><i class="fas fa-robot"></i> Servo Control (0–180°, center at 90°)</div>
                <div class="panel-content">
                    <div class="servo-control">
                        <label><i class="fas fa-sync"></i> Base (ID 6)</label>
                        <input type="range" id="servo6" min="0" max="180" value="90" oninput="updateServo(6, this.value)">
                        <input class="servo-value" id="servo6-input" type="number" step="1" min="0" max="180" value="90" onkeydown="onServoInputKey(event, 6)">
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-arrows-alt-v"></i> Shoulder (ID 5)</label>
                        <input type="range" id="servo5" min="0" max="180" value="90" oninput="updateServo(5, this.value)">
                        <input class="servo-value" id="servo5-input" type="number" step="1" min="0" max="180" value="90" onkeydown="onServoInputKey(event, 5)">
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-angle-double-right"></i> Elbow (ID 4)</label>
                        <input type="range" id="servo4" min="0" max="180" value="90" oninput="updateServo(4, this.value)">
                        <input class="servo-value" id="servo4-input" type="number" step="1" min="0" max="180" value="90" onkeydown="onServoInputKey(event, 4)">
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-hand-paper"></i> Wrist (ID 3)</label>
                        <input type="range" id="servo3" min="0" max="180" value="90" oninput="updateServo(3, this.value)">
                        <input class="servo-value" id="servo3-input" type="number" step="1" min="0" max="180" value="90" onkeydown="onServoInputKey(event, 3)">
                    </div>
                    <div class="servo-control">
                        <label><i class="fas fa-grip-lines"></i> Gripper (ID 1)</label>
                        <input type="range" id="servo1" min="0" max="180" value="90" oninput="updateServo(1, this.value)">
                        <input class="servo-value" id="servo1-input" type="number" step="1" min="0" max="180" value="90" onkeydown="onServoInputKey(event, 1)">
                    </div>
                </div>
            </div>
            
            <!-- Sidebar command log removed: moved below camera for always-visible large log -->
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
         // Edited angles queue for batched send on Enter
         const editedAngles = {};
         let servoIdsFromConfig = [];

         async function updateServo(servoNum, angleDeg) {
            const angle = parseInt(angleDeg);
            const range = document.getElementById(`servo${servoNum}`);
            const input = document.getElementById(`servo${servoNum}-input`);
            if (range) range.value = angle;
            if (input) input.value = angle;
            logMessage(`🦾 Moving servo ${servoNum} to ${angle}°`, 'system');
            // Backend supports 0–180 directly; use 1000ms smoothing time
            const result = await sendRPC('SetPWMServo', [1000, servoNum, angle]);
            if (result && result.result) {
                logMessage(`✅ Servo ${servoNum} moved to ${angle}°`, 'success');
            }
        }

        function onServoInputKey(e, servoNum){
            if (e.key !== 'Enter') return;
            const input = document.getElementById(`servo${servoNum}-input`);
            const slider = document.getElementById(`servo${servoNum}`);
            if (!input || !slider) return;
            const min = parseInt(slider.min);
            const max = parseInt(slider.max);
            let angle = parseInt(input.value);
            if (isNaN(angle)) angle = min;
            if (angle < min) angle = min;
            if (angle > max) angle = max;
            input.value = angle;
            // Queue this edit
            editedAngles[servoNum] = angle;
            // Send all queued edits in one batch
            sendBatchEdited();
        }

        async function sendBatchEdited(){
            const ids = Object.keys(editedAngles);
            if (ids.length === 0) return;
            // Build params: [duration_ms, id1, angle1, id2, angle2, ...]
            const params = [1000];
            ids.forEach(k => {
                params.push(parseInt(k), parseInt(editedAngles[k]));
            });
            logMessage(`🧩 Batch moving servos: ${ids.join(', ')}`, 'system');
            const result = await sendRPC('SetPWMServo', params);
            if (result && result.result) {
                logMessage('✅ Batch move complete', 'success');
                // Sync sliders/inputs and clear queue
                ids.forEach(k => {
                    const range = document.getElementById(`servo${k}`);
                    const input = document.getElementById(`servo${k}-input`);
                    if (range) range.value = editedAngles[k];
                    if (input) input.value = editedAngles[k];
                });
                // Clear edits
                for (const k of ids) delete editedAngles[k];
            } else {
                logMessage('❌ Batch move failed', 'error');
            }
        }

        async function fetchServoConfig() {
            try {
                const res = await fetch('/api/servo_config');
                const cfg = await res.json();
                if (!cfg || !cfg.servos) return;
                servoIdsFromConfig = [];
                cfg.servos.forEach(s => {
                    const id = s.id;
                    servoIdsFromConfig.push(id);
                    const degMin = s.degrees_min ?? 0;
                    const degMax = s.degrees_max ?? 180;
                    const defDeg = s.default_degrees ?? Math.round((degMin + degMax)/2);
                    const slider = document.getElementById(`servo${id}`);
                    const input  = document.getElementById(`servo${id}-input`);
                    if (slider) {
                        slider.min = degMin;
                        slider.max = degMax;
                        slider.value = defDeg;
                    }
                    if (input) input.value = defDeg;
                });
            } catch(e) {
                console.error('Failed to load servo config', e);
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
            fetchServoConfig();
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

HTML_3D_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🤖 ArmPi Mini 3D Model Viewer</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" rel="stylesheet">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
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
             text-align: left;
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
        <h1>ArmPi Mini 3D Model Viewer</h1>
        <div style="margin-top:0.5rem">
            <a href="/" class="btn btn-primary" style="text-decoration:none; padding:0.6rem 1rem; border-radius:10px; display:inline-block"><i class="fas fa-arrow-left"></i> Back to Camera</a>
            <a href="/combined" class="btn btn-secondary" style="text-decoration:none; padding:0.6rem 1rem; border-radius:10px; display:inline-block; margin-left:0.5rem"><i class="fas fa-columns"></i> Combined View</a>
        </div>
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
                <h3>🎮 3D Model Viewer</h3>
            </div>
            <div class="video-container">
                                 <div id="3dCanvasContainer" style="width: 100%; height: 400px; border-radius: 15px; box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2); border: 2px solid rgba(255, 255, 255, 0.1); position: relative; background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);"></div>
                 <div style="margin-top: 1rem; text-align: center;">
                     <div class="button-grid">
                         <button class="btn btn-primary" onclick="loadModel('armpi_mini.stl')">
                             <i class="fas fa-robot"></i> Full Robot
                         </button>
                         <button class="btn btn-secondary" onclick="loadModel('armpi_mini.3mf')">
                             <i class="fas fa-cube"></i> 3MF Model
                         </button>
                         <button class="btn btn-success" onclick="resetCamera()">
                             <i class="fas fa-redo"></i> Reset View
                         </button>
                         <button class="btn btn-warning" onclick="toggleWireframe()">
                             <i class="fas fa-eye"></i> Wireframe
                         </button>
                     </div>
                     <div style="margin-top: 1rem;">
                         <div style="margin-bottom: 1rem;">
                             <strong style="color: rgba(255,255,255,0.9);">Material Colors:</strong>
                             <div style="display: flex; gap: 0.5rem; margin-top: 0.5rem; justify-content: center;">
                                 <button class="color-btn" style="background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);" onclick="changeMaterialColor(0x4facfe)" title="Blue"></button>
                                 <button class="color-btn" style="background: linear-gradient(135deg, #43e97b 0%, #38f9d7 100%);" onclick="changeMaterialColor(0x43e97b)" title="Green"></button>
                                 <button class="color-btn" style="background: linear-gradient(135deg, #fa709a 0%, #fee140 100%);" onclick="changeMaterialColor(0xfa709a)" title="Pink"></button>
                                 <button class="color-btn" style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);" onclick="changeMaterialColor(0x667eea)" title="Purple"></button>
                                 <button class="color-btn" style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);" onclick="changeMaterialColor(0xf093fb)" title="Red"></button>
                                 <button class="color-btn" style="background: linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%);" onclick="changeMaterialColor(0xffecd2)" title="Orange"></button>
                             </div>
                         </div>
                     </div>
                     <div style="margin-top: 1rem; color: rgba(255,255,255,0.8); font-size: 0.9rem;">
                         <i class="fas fa-info-circle"></i> Use mouse to rotate, scroll to zoom, right-click to pan
                     </div>
                 </div>
            </div>
        </div>
        
        <div class="controls-section">
            <div class="control-panel">
                <div class="panel-header"><i class="fas fa-gamepad"></i> Servo Control</div>
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
                 <div class="panel-header"><i class="fas fa-cog"></i> 3D Controls</div>
                 <div class="panel-content">
                     <div class="button-grid">
                         <button class="btn btn-secondary" onclick="toggleAutoRotate()">
                             <i class="fas fa-sync-alt"></i> Auto Rotate
                         </button>
                         <button class="btn btn-secondary" onclick="toggleLighting()">
                             <i class="fas fa-lightbulb"></i> Lighting
                         </button>
                     </div>
                     <div style="margin-top: 1rem;">
                         <strong style="color: rgba(255,255,255,0.9);">Model Info:</strong>
                         <div id="modelInfo" style="margin-top: 0.5rem; padding: 1rem; background: rgba(255,255,255,0.05); border-radius: 10px; font-size: 0.9rem; color: rgba(255,255,255,0.7);">
                             No model loaded
                         </div>
                     </div>
                 </div>
             </div>
             
             <div class="control-panel">
                 <div class="panel-header"><i class="fas fa-terminal"></i> Command Log</div>
                 <div class="panel-content">
                     <div class="log-output" id="logOutput">
                         <span style="color: #00ff88;">[SYSTEM]</span> 3D model viewer loaded successfully...<br>
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
        
        // Servo control
        async function updateServo(servoNum, angle) {
            document.getElementById(`servo${servoNum}-value`).textContent = `${angle}°`;
            logMessage(`🦾 Moving servo ${servoNum} to ${angle}°`, 'system');
            const result = await sendRPC('SetPWMServo', [1000, servoNum, parseInt(angle)]);
            if (result && result.result) {
                logMessage(`✅ Servo ${servoNum} moved to ${angle}°`, 'success');
            }
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
        
                 // Three.js 3D Viewer Variables
         let scene, camera, renderer, controls, currentModel;
         let isWireframe = false;
         
         // Initialize 3D Viewer
         function init3DViewer() {
             const container = document.getElementById('3dCanvasContainer');
             
             // Scene
             scene = new THREE.Scene();
             scene.background = new THREE.Color(0x1a1a2e);
             
             // Camera
             camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
             camera.position.set(0, 0, 100);
             
             // Renderer
             renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
             renderer.setSize(container.clientWidth, container.clientHeight);
             renderer.setPixelRatio(window.devicePixelRatio);
             renderer.shadowMap.enabled = true;
             renderer.shadowMap.type = THREE.PCFSoftShadowMap;
             container.appendChild(renderer.domElement);
             
             // Controls
             controls = new THREE.OrbitControls(camera, renderer.domElement);
             controls.enableDamping = true;
             controls.dampingFactor = 0.05;
             controls.screenSpacePanning = false;
             controls.maxPolarAngle = Math.PI;
             
             // Lighting
             const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
             scene.add(ambientLight);
             
             const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
             directionalLight.position.set(1, 1, 1);
             directionalLight.castShadow = true;
             scene.add(directionalLight);
             
             const pointLight1 = new THREE.PointLight(0x4facfe, 0.5, 100);
             pointLight1.position.set(50, 50, 50);
             scene.add(pointLight1);
             
             const pointLight2 = new THREE.PointLight(0xfe466b, 0.3, 100);
             pointLight2.position.set(-50, -50, 50);
             scene.add(pointLight2);
             
             // Add a grid
             const gridHelper = new THREE.GridHelper(200, 50, 0x444444, 0x444444);
             scene.add(gridHelper);
             
             // Animation loop
             animate();
             
             logMessage('🎮 3D Viewer initialized successfully!', 'success');
         }
         
         // Load 3D Model
         async function loadModel(filename) {
             try {
                 logMessage(`📦 Loading model: ${filename}`, 'system');
                 
                 // Remove existing model
                 if (currentModel) {
                     scene.remove(currentModel);
                 }
                 
                 // Show loading indicator
                 const container = document.getElementById('3dCanvasContainer');
                 const loadingDiv = document.createElement('div');
                 loadingDiv.id = 'loadingIndicator';
                 loadingDiv.style.cssText = `
                     position: absolute;
                     top: 50%;
                     left: 50%;
                     transform: translate(-50%, -50%);
                     color: white;
                     font-size: 1.2rem;
                     z-index: 1000;
                 `;
                 loadingDiv.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Loading 3D Model...';
                 container.appendChild(loadingDiv);
                 
                 if (filename.endsWith('.stl')) {
                     // Load STL file
                     const loader = new THREE.STLLoader();
                     loader.load(`/models/${filename}`, function(geometry) {
                         // Create material
                         const material = new THREE.MeshPhongMaterial({
                             color: 0x4facfe,
                             shininess: 100,
                             transparent: true,
                             opacity: 0.9
                         });
                         
                         // Create mesh
                         const mesh = new THREE.Mesh(geometry, material);
                         
                         // Center and scale the model
                         geometry.computeBoundingBox();
                         const box = geometry.boundingBox;
                         const center = box.getCenter(new THREE.Vector3());
                         geometry.translate(-center.x, -center.y, -center.z);
                         
                         const size = box.getSize(new THREE.Vector3());
                         const maxDim = Math.max(size.x, size.y, size.z);
                         const scale = 50 / maxDim;
                         mesh.scale.setScalar(scale);
                         
                         mesh.castShadow = true;
                         mesh.receiveShadow = true;
                         
                         // Add to scene
                         scene.add(mesh);
                         currentModel = mesh;
                         
                         // Update model information
                         updateModelInfo(filename, geometry);
                         
                         // Remove loading indicator
                         const loadingIndicator = document.getElementById('loadingIndicator');
                         if (loadingIndicator) {
                             loadingIndicator.remove();
                         }
                         
                         logMessage(`✅ Successfully loaded ${filename}`, 'success');
                     }, function(progress) {
                         console.log('Loading progress:', progress);
                     }, function(error) {
                         console.error('Error loading STL:', error);
                         logMessage(`❌ Failed to load ${filename}: ${error.message}`, 'error');
                         const loadingIndicator = document.getElementById('loadingIndicator');
                         if (loadingIndicator) {
                             loadingIndicator.remove();
                         }
                     });
                 } else {
                     logMessage(`❌ Unsupported file format: ${filename}`, 'error');
                     const loadingIndicator = document.getElementById('loadingIndicator');
                     if (loadingIndicator) {
                         loadingIndicator.remove();
                     }
                 }
                 
             } catch (error) {
                 logMessage(`❌ Error loading model: ${error.message}`, 'error');
                 console.error('Model loading error:', error);
             }
         }
         
         // Reset camera to default position
         function resetCamera() {
             camera.position.set(0, 0, 100);
             controls.reset();
             logMessage('📷 Camera view reset', 'system');
         }
         
         // Toggle wireframe mode
         function toggleWireframe() {
             if (currentModel && currentModel.material) {
                 isWireframe = !isWireframe;
                 currentModel.material.wireframe = isWireframe;
                 logMessage(`🔧 Wireframe mode: ${isWireframe ? 'ON' : 'OFF'}`, 'system');
             } else {
                 logMessage('❌ No model loaded to toggle wireframe', 'warning');
             }
         }
         
         // Change material color
         function changeMaterialColor(color) {
             if (currentModel && currentModel.material) {
                 currentModel.material.color.setHex(color);
                 logMessage(`🎨 Material color changed to #${color.toString(16).padStart(6, '0')}`, 'system');
             } else {
                 logMessage('❌ No model loaded to change color', 'warning');
             }
         }
         
         // Auto-rotate toggle
         let autoRotate = false;
         function toggleAutoRotate() {
             autoRotate = !autoRotate;
             if (autoRotate && currentModel) {
                 logMessage('🔄 Auto-rotation enabled', 'system');
             } else {
                 logMessage('⏸️ Auto-rotation disabled', 'system');
             }
         }
         
         // Lighting toggle
         let lightingEnabled = true;
         function toggleLighting() {
             lightingEnabled = !lightingEnabled;
             scene.children.forEach(child => {
                 if (child.type === 'DirectionalLight' || child.type === 'PointLight') {
                     child.visible = lightingEnabled;
                 }
             });
             logMessage(`💡 Lighting: ${lightingEnabled ? 'ON' : 'OFF'}`, 'system');
         }
         
         // Update model information
         function updateModelInfo(filename, geometry) {
             const vertices = geometry.attributes.position.count;
             const faces = vertices / 3;
             const size = new THREE.Vector3();
             geometry.computeBoundingBox();
             geometry.boundingBox.getSize(size);
             
             const info = `
                 <strong>📄 File:</strong> ${filename}<br>
                 <strong>📊 Vertices:</strong> ${vertices.toLocaleString()}<br>
                 <strong>🔺 Faces:</strong> ${Math.floor(faces).toLocaleString()}<br>
                 <strong>📏 Size:</strong> ${size.x.toFixed(1)} × ${size.y.toFixed(1)} × ${size.z.toFixed(1)}<br>
                 <strong>🎨 Material:</strong> Phong Shading<br>
                 <strong>✨ Features:</strong> Shadows, Anti-aliasing
             `;
             
             document.getElementById('modelInfo').innerHTML = info;
         }
         
         // Enhanced animation loop with auto-rotation
         function animate() {
             requestAnimationFrame(animate);
             
             // Auto-rotate model if enabled
             if (autoRotate && currentModel) {
                 currentModel.rotation.y += 0.01;
             }
             
             controls.update();
             renderer.render(scene, camera);
         }
         
         // Handle window resize
         function handleResize() {
             const container = document.getElementById('3dCanvasContainer');
             camera.aspect = container.clientWidth / container.clientHeight;
             camera.updateProjectionMatrix();
             renderer.setSize(container.clientWidth, container.clientHeight);
         }
         
         window.addEventListener('resize', handleResize);
         
         // Initialize
         document.addEventListener('DOMContentLoaded', function() {
             init3DViewer();
             logMessage('🚀 3D Model Viewer initialized successfully!');
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
            logMessage('🤖 Welcome to ArmPi Mini 3D Model Viewer!', 'system');
            logMessage('💡 Click buttons to load models', 'info');
            logMessage('🎮 Use sliders to move servos in real-time', 'info');
        }, 1000);
    </script>
</body>
</html>
'''

HTML_COMBINED_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🤖 ArmPi Mini — Combined View</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" rel="stylesheet">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: #fff; margin:0 }
        .header { padding: 1rem; text-align:center }
        .container { max-width: 1280px; margin: 0 auto; padding: 1rem }
        .card { background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.15); border-radius: 16px; padding: 1rem; box-shadow: 0 10px 30px rgba(0,0,0,0.2); margin-bottom: 1rem }
        .btn { padding: .6rem 1rem; border-radius:10px; border:1px solid rgba(255,255,255,0.25); color:#fff; text-decoration:none; display:inline-block }
        .btn + .btn { margin-left:.5rem }
        .btn-primary { background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%) }
        .btn-secondary { background: rgba(255,255,255,0.15) }
        #combined3dCanvasContainer { width: 100%; height: 420px; border-radius: 12px; background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%); border: 2px solid rgba(255,255,255,0.1) }
    </style>
 </head>
 <body>
   <div class="header">
     <h2>ArmPi Mini — Combined View</h2>
     <div>
       <a class="btn btn-primary" href="/"><i class="fas fa-video"></i> Camera</a>
       <a class="btn btn-secondary" href="/3d"><i class="fas fa-cube"></i> 3D Viewer</a>
     </div>
   </div>
   <div class="container">
     <div class="card">
       <h3 style="margin:0 0 .5rem 0">📹 Live Camera</h3>
       <img id="combinedVideo" src="/video_feed" style="max-width:100%; border-radius:12px; border:2px solid rgba(255,255,255,0.1)" />
     </div>
     <div class="card">
       <h3 style="margin:0 0 .5rem 0">🎮 3D Model</h3>
       <div id="combined3dCanvasContainer"></div>
       <div style="margin-top:.6rem">
          <a class="btn btn-secondary" href="#" onclick="loadModel2('armpi_mini.stl')"><i class="fas fa-robot"></i> Full Robot</a>
          <a class="btn btn-secondary" href="#" onclick="resetCamera2()"><i class="fas fa-redo"></i> Reset View</a>
          <a class="btn btn-secondary" href="#" onclick="toggleWireframe2()"><i class="fas fa-eye"></i> Wireframe</a>
       </div>
     </div>
   </div>
   <script>
     let scene2, camera2, renderer2, controls2, currentModel2, autoRotate2=false, isWire2=false;
     function init3DViewer2(){
        const c = document.getElementById('combined3dCanvasContainer');
        scene2 = new THREE.Scene();
        scene2.background = new THREE.Color(0x1a1a2e);
        camera2 = new THREE.PerspectiveCamera(75, c.clientWidth/c.clientHeight, .1, 1000);
        camera2.position.set(0,0,100);
        renderer2 = new THREE.WebGLRenderer({antialias:true, alpha:true});
        renderer2.setSize(c.clientWidth, c.clientHeight);
        c.appendChild(renderer2.domElement);
        controls2 = new THREE.OrbitControls(camera2, renderer2.domElement);
        scene2.add(new THREE.AmbientLight(0x404040, .5));
        const d = new THREE.DirectionalLight(0xffffff,.9); d.position.set(1,1,1); scene2.add(d);
        scene2.add(new THREE.GridHelper(200,50,0x444444,0x444444));
        function animate(){ requestAnimationFrame(animate); if(autoRotate2&&currentModel2){ currentModel2.rotation.y+=.01;} controls2.update(); renderer2.render(scene2,camera2);} animate();
     }
     function loadModel2(filename){
        const c = document.getElementById('combined3dCanvasContainer');
        if(currentModel2){ scene2.remove(currentModel2); }
        const loader = new THREE.STLLoader();
        loader.load(`/models/${filename}`, (geometry)=>{
          const mat = new THREE.MeshPhongMaterial({color:0x4facfe, shininess:100, transparent:true, opacity:0.95});
          const mesh = new THREE.Mesh(geometry, mat);
          geometry.computeBoundingBox(); const box = geometry.boundingBox; const center = box.getCenter(new THREE.Vector3()); geometry.translate(-center.x,-center.y,-center.z);
          const size = box.getSize(new THREE.Vector3()); const maxDim = Math.max(size.x,size.y,size.z); mesh.scale.setScalar(50/maxDim);
          scene2.add(mesh); currentModel2 = mesh;
        });
     }
     function resetCamera2(){ camera2.position.set(0,0,100); controls2.reset(); }
     function toggleWireframe2(){ if(currentModel2){ isWire2=!isWire2; currentModel2.material.wireframe=isWire2; } }
     window.addEventListener('resize', ()=>{ const c=document.getElementById('combined3dCanvasContainer'); if(!c||!camera2||!renderer2)return; camera2.aspect=c.clientWidth/c.clientHeight; camera2.updateProjectionMatrix(); renderer2.setSize(c.clientWidth,c.clientHeight); });
     document.addEventListener('DOMContentLoaded', ()=>{ init3DViewer2(); });
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