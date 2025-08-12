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
    # Provide comprehensive servo configuration
    out = {"servos": [], "config_loaded": bool(SERVO_MAP), "timestamp": time.time()}
    seen = set()
    for s in cfg.get('servos', []):
        sid = int(s.get('id', -1))
        if sid <= 0 or sid in seen:
            continue
        seen.add(sid)
        deg_min = int(s.get('degrees_min', 0))
        deg_max = int(s.get('degrees_max', 180))
        default_deg = int(s.get('default_degrees', (deg_min + deg_max)//2))
        
        out['servos'].append({
            "id": sid,
            "name": s.get('name', f'Servo {sid}'),
            "degrees_min": deg_min,
            "degrees_max": deg_max,
            "default_degrees": default_deg,
            "pulse_min": int(s.get('pulse_min', 500)),
            "pulse_max": int(s.get('pulse_max', 2500)),
            "invert": bool(s.get('invert', False)),
            "offset_pulse": int(s.get('offset_pulse', 0))
        })
    
    # Sort by ID for consistent ordering
    out['servos'].sort(key=lambda x: x['id'])
    return jsonify(out)

@app.route('/api/servo_config/reload', methods=['POST'])
def reload_servo_config():
    """Reload servo configuration from YAML file"""
    global SERVO_MAP
    try:
        SERVO_MAP = load_servo_map()
        return jsonify({
            "success": True, 
            "message": "Servo configuration reloaded successfully",
            "config_loaded": bool(SERVO_MAP),
            "servo_count": len(SERVO_MAP.get('servos', [])) if SERVO_MAP else 0
        })
    except Exception as e:
        return jsonify({
            "success": False, 
            "message": f"Failed to reload servo configuration: {str(e)}"
        }), 500

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
                <div class="panel-header">
                    <div style="display: flex; align-items: center; justify-content: space-between;">
                        <span id="servoControlTitle"><i class="fas fa-robot"></i> Servo Control (Loading...)</span>
                        <div>
                            <button class="btn btn-secondary" onclick="reloadServoConfig()" style="font-size: 0.8rem; padding: 0.3rem 0.8rem; margin-right: 0.5rem;" title="Reload servo config from YAML">
                                <i class="fas fa-sync"></i> Reload
                            </button>
                            <button class="btn btn-secondary" onclick="resetAllServos()" style="font-size: 0.8rem; padding: 0.3rem 0.8rem;" title="Reset all servos to center">
                                <i class="fas fa-home"></i> Reset All
                            </button>
                        </div>
                    </div>
                </div>
                <div class="panel-content">
                    <div id="servoConfigStatus" style="margin-bottom: 1rem; padding: 0.5rem; border-radius: 5px; font-size: 0.85rem; display: none;">
                        <i class="fas fa-info-circle"></i> <span id="configStatusText">Loading servo configuration...</span>
                    </div>
                    <div id="servoControlsContainer" style="display: grid; gap: 1rem;">
                        <!-- Servo controls will be dynamically generated here -->
                        <div style="text-align: center; color: rgba(255,255,255,0.7); padding: 2rem;">
                            <i class="fas fa-spinner fa-spin" style="font-size: 1.5rem; margin-bottom: 0.5rem;"></i><br>
                            Loading servo configuration...
                        </div>
                    </div>
                    <div style="margin-top: 1rem; padding: 0.8rem; background: rgba(67, 233, 123, 0.1); border-radius: 8px; font-size: 0.85rem;">
                        <i class="fas fa-info-circle" style="color: #43e97b; margin-right: 0.5rem;"></i>
                        <strong>Batch Control:</strong> Adjust multiple servos, then press <kbd style="background: rgba(255,255,255,0.2); padding: 0.2rem 0.4rem; border-radius: 3px;">Enter</kbd> in any field to move all modified servos simultaneously.
                        <br><small style="color: rgba(255,255,255,0.6); margin-top: 0.3rem; display: block;">
                            Servo ranges are automatically loaded from <code>config/servo_map.yaml</code>
                        </small>
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
        
        // Color tracking with animation sync
        async function setTargetColor(color) {
            logMessage(`🎨 Setting target color to ${color}...`, 'system');
            const result = await sendRPC('ColorTracking', [color]);
            if (result && result.result) {
                logMessage(`✅ Target color set to ${color}`, 'success');
                
                // Sync landing page animation colors
                updateAnimationColors(color);
            }
        }
        
        // Update animation colors based on selected color
        function updateAnimationColors(color) {
            const colorMappings = {
                'red': {
                    primary: 'linear-gradient(135deg, #e53e3e 0%, #ff6b6b 100%)',
                    secondary: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
                    accent: 'linear-gradient(135deg, #fc466b 0%, #3f5efb 100%)'
                },
                'green': {
                    primary: 'linear-gradient(135deg, #38a169 0%, #48bb78 100%)',
                    secondary: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
                    accent: 'linear-gradient(135deg, #00f2fe 0%, #4facfe 100%)'
                },
                'blue': {
                    primary: 'linear-gradient(135deg, #3182ce 0%, #4299e1 100%)',
                    secondary: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
                    accent: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
                },
                'yellow': {
                    primary: 'linear-gradient(135deg, #d69e2e 0%, #ecc94b 100%)',
                    secondary: 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)',
                    accent: 'linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%)'
                },
                'purple': {
                    primary: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                    secondary: 'linear-gradient(135deg, #805ad5 0%, #9f7aea 100%)',
                    accent: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)'
                },
                'orange': {
                    primary: 'linear-gradient(135deg, #dd6b20 0%, #ed8936 100%)',
                    secondary: 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)',
                    accent: 'linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%)'
                }
            };
            
            const selectedColors = colorMappings[color] || colorMappings['purple']; // fallback to purple
            
            // Update CSS custom properties for smooth transition
            const root = document.documentElement;
            root.style.setProperty('--primary-gradient', selectedColors.primary);
            root.style.setProperty('--secondary-gradient', selectedColors.secondary);
            root.style.setProperty('--accent-gradient', selectedColors.accent);
            
            // Animate the body background
            document.body.style.background = selectedColors.primary;
            
            // Update active color button visual feedback
            document.querySelectorAll('.color-btn').forEach(btn => btn.classList.remove('active'));
            const activeBtn = document.querySelector(`.color-btn.${color}`);
            if (activeBtn) {
                activeBtn.classList.add('active');
                activeBtn.style.transform = 'scale(1.1)';
                activeBtn.style.boxShadow = '0 0 20px rgba(255,255,255,0.4)';
                
                // Reset scale after animation
                setTimeout(() => {
                    activeBtn.style.transform = '';
                    activeBtn.style.boxShadow = '';
                }, 300);
            }
            
            logMessage(`🌈 Animation colors updated to ${color} theme`, 'info');
        }
        
        // Servo control - Improved batching system
         const editedAngles = {};
         let servoIdsFromConfig = [];
         let batchTimeout = null;
         const BATCH_DELAY = 500; // ms to wait before sending batch
         
         // Enhanced camera simulation for realistic servo movement visualization
         let isTrackingCamera = false;
         let previousServoPositions = {};
         
         function updateCameraPosition() {
             if (!currentModel) return;
             
             // Get current servo positions using correct servo IDs
             const servo6 = parseInt(document.getElementById('servo6')?.value || 90); // Base
             const servo5 = parseInt(document.getElementById('servo5')?.value || 90); // Shoulder
             const servo4 = parseInt(document.getElementById('servo4')?.value || 90); // Elbow
             const servo3 = parseInt(document.getElementById('servo3')?.value || 90); // Wrist
             const servo1 = parseInt(document.getElementById('servo1')?.value || 90); // Gripper
             
             // Check if any servos actually moved
             const currentPositions = { servo6, servo5, servo4, servo3, servo1 };
             const hasMovement = Object.keys(currentPositions).some(key => 
                 previousServoPositions[key] !== currentPositions[key]
             );
             
             if (hasMovement && currentModel) {
                 // Convert servo angles to radians (center at 90°)
                 const baseRotation = (servo6 - 90) * (Math.PI / 180) * 0.8; // Base rotation
                 const shoulderTilt = (servo5 - 90) * (Math.PI / 180) * 0.2; // Shoulder influence
                 const elbowFine = (servo4 - 90) * (Math.PI / 180) * 0.1; // Elbow fine adjustment
                 
                 // Smoothly animate the model to new positions
                 const animateToPosition = (startTime) => {
                     const elapsed = Date.now() - startTime;
                     const duration = 800; // ms for smooth animation
                     const progress = Math.min(elapsed / duration, 1);
                     const easeProgress = 1 - Math.pow(1 - progress, 2); // easeOutQuad
                     
                     // Store original rotation at start of animation
                     if (!currentModel.originalRotation) {
                         currentModel.originalRotation = { 
                             x: currentModel.rotation.x,
                             y: currentModel.rotation.y,
                             z: currentModel.rotation.z
                         };
                     }
                     
                     // Calculate target rotations
                     const targetY = currentModel.originalRotation.y + baseRotation;
                     const targetX = currentModel.originalRotation.x + shoulderTilt;
                     const targetZ = currentModel.originalRotation.z + elbowFine;
                     
                     // Interpolate to target positions
                     currentModel.rotation.y = currentModel.originalRotation.y + 
                         (targetY - currentModel.originalRotation.y) * easeProgress;
                     currentModel.rotation.x = currentModel.originalRotation.x + 
                         (targetX - currentModel.originalRotation.x) * easeProgress;
                     currentModel.rotation.z = currentModel.originalRotation.z + 
                         (targetZ - currentModel.originalRotation.z) * easeProgress;
                     
                     // Dynamic camera positioning to follow the action
                     if (camera && controls) {
                         const radius = 6;
                         const cameraAngle = currentModel.rotation.y * 0.3;
                         
                         // Calculate new camera position
                         const targetCameraX = Math.sin(cameraAngle) * radius;
                         const targetCameraZ = Math.cos(cameraAngle) * radius;
                         const targetCameraY = 3 + Math.sin(shoulderTilt) * 2;
                         
                         // Smooth camera movement
                         camera.position.x += (targetCameraX - camera.position.x) * 0.05;
                         camera.position.z += (targetCameraZ - camera.position.z) * 0.05;
                         camera.position.y += (targetCameraY - camera.position.y) * 0.03;
                         
                         // Keep camera looking at the model center
                         controls.target.set(0, 1, 0);
                         controls.update();
                     }
                     
                     // Continue animation until complete
                     if (progress < 1) {
                         requestAnimationFrame(() => animateToPosition(startTime));
                     } else {
                         // Clear original rotation for next movement
                         currentModel.originalRotation = null;
                         logMessage(`📷 Camera animation complete - servos: Base:${servo6}°, Shoulder:${servo5}°, Elbow:${servo4}°`, 'system');
                     }
                 };
                 
                 // Start animation
                 animateToPosition(Date.now());
                 logMessage(`🎬 Animating camera for servo movement: Base:${servo6}°, Shoulder:${servo5}°`, 'info');
             }
             
             // Store current positions for next comparison
             previousServoPositions = currentPositions;
         }
         
         // Auto-start camera tracking on servo interaction
         function startCameraTracking() {
             if (!isTrackingCamera) {
                 isTrackingCamera = true;
                 logMessage('📹 Camera tracking enabled', 'info');
             }
         }
         
         function stopCameraTracking() {
             isTrackingCamera = false;
             logMessage('📹 Camera tracking disabled', 'info');
         }

         async function updateServo(servoNum, angleDeg) {
            let angle = parseInt(angleDeg);
            
            // Validate against servo configuration limits
            if (servoConfigData && servoConfigData.servos) {
                const servoConfig = servoConfigData.servos.find(s => s.id === servoNum);
                if (servoConfig) {
                    // Clamp to valid range
                    angle = Math.max(servoConfig.degrees_min, Math.min(servoConfig.degrees_max, angle));
                    
                    // Warn if value was clamped
                    if (angle !== parseInt(angleDeg)) {
                        logMessage(`⚠️ Servo ${servoNum} clamped from ${angleDeg}° to ${angle}° (range: ${servoConfig.degrees_min}°-${servoConfig.degrees_max}°)`, 'warning');
                    }
                }
            }
            
            const range = document.getElementById(`servo${servoNum}`);
            const input = document.getElementById(`servo${servoNum}-input`);
            const valueDisplay = document.getElementById(`servo${servoNum}-value`);
            
            if (range) range.value = angle;
            if (input) input.value = angle;
            if (valueDisplay) valueDisplay.textContent = `${angle}°`;
            
            // Queue this change for batching
            editedAngles[servoNum] = angle;
            
            // Add visual feedback that servo is queued for update
            const servoControl = range ? range.closest('.servo-control') : null;
            if (servoControl) {
                servoControl.style.borderLeft = '3px solid #4facfe';
                setTimeout(() => {
                    servoControl.style.borderLeft = '';
                }, 500);
            }
            
            // Show pending status
            logMessage(`📝 Servo ${servoNum} queued: ${angle}° (pending batch)`, 'system');
            
            // Schedule batch send
            scheduleBatchSend();
        }
        
                 // Reset all servos to their default positions from config
         async function resetAllServos() {
             if (!servoConfigData || !servoConfigData.servos) {
                 logMessage('❌ No servo configuration available for reset', 'error');
                 return;
             }
             
             logMessage('🏠 Resetting all servos to default positions...', 'system');
             
             // Reset all UI elements using config defaults
             servoConfigData.servos.forEach(servo => {
                 const range = document.getElementById(`servo${servo.id}`);
                 const input = document.getElementById(`servo${servo.id}-input`);
                 const valueDisplay = document.getElementById(`servo${servo.id}-value`);
                 
                 if (range) range.value = servo.default_degrees;
                 if (input) input.value = servo.default_degrees;
                 if (valueDisplay) valueDisplay.textContent = `${servo.default_degrees}°`;
                 
                 editedAngles[servo.id] = servo.default_degrees;
             });
             
             // Send batch command to reset all servos
             clearTimeout(batchTimeout);
             await sendBatchEdited();
             
             logMessage('✅ All servos reset to default positions', 'success');
         }
         
         // Reload servo configuration from YAML
         async function reloadServoConfig() {
             try {
                 showConfigStatus('🔄 Reloading servo configuration...', 'info');
                 logMessage('🔄 Reloading servo configuration from YAML...', 'system');
                 
                 const response = await fetch('/api/servo_config/reload', {
                     method: 'POST',
                     headers: {
                         'Content-Type': 'application/json'
                     }
                 });
                 
                 const result = await response.json();
                 
                 if (result.success) {
                     logMessage(`✅ ${result.message} (${result.servo_count} servos)`, 'success');
                     showConfigStatus(`✅ ${result.message}`, 'success');
                     
                     // Fetch the updated configuration
                     await fetchServoConfig();
                 } else {
                     logMessage(`❌ ${result.message}`, 'error');
                     showConfigStatus(`❌ ${result.message}`, 'error');
                 }
             } catch (error) {
                 const errorMsg = `Failed to reload servo config: ${error.message}`;
                 logMessage(`❌ ${errorMsg}`, 'error');
                 showConfigStatus(`❌ ${errorMsg}`, 'error');
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
            slider.value = angle; // Also update the slider visual
            
            // Update servo immediately on Enter
            editedAngles[servoNum] = angle;
            
            // Force immediate batch send on Enter - this will send ALL pending servos
            clearTimeout(batchTimeout);
            sendBatchEdited();
            
            // Log which servo triggered the batch send for clarity
            logMessage(`🎯 Servo ${servoNum} triggered batch send (Enter pressed)`, 'info');
        }

        function scheduleBatchSend() {
            // Clear existing timeout
            clearTimeout(batchTimeout);
            
            // Schedule new batch send
            batchTimeout = setTimeout(() => {
                sendBatchEdited();
            }, BATCH_DELAY);
        }

        async function sendBatchEdited(){
            const ids = Object.keys(editedAngles);
            if (ids.length === 0) return;
            
            // Clear timeout since we're sending now
            clearTimeout(batchTimeout);
            
            // Build params: [duration_ms, id1, angle1, id2, angle2, ...]
            const params = [1000];
            const servoDetails = [];
            ids.forEach(k => {
                params.push(parseInt(k), parseInt(editedAngles[k]));
                servoDetails.push(`Servo ${k}: ${editedAngles[k]}°`);
            });
            
            logMessage(`🧩 Batch moving ${ids.length} servo(s): ${servoDetails.join(', ')}`, 'system');
            const result = await sendRPC('SetPWMServo', params);
            
            if (result && result.result) {
                logMessage(`✅ Batch move complete: ${servoDetails.join(', ')}`, 'success');
                
                // Sync all servo displays with visual feedback
                ids.forEach(k => {
                    const range = document.getElementById(`servo${k}`);
                    const input = document.getElementById(`servo${k}-input`);
                    const valueDisplay = document.getElementById(`servo${k}-value`);
                    const servoControl = range ? range.closest('.servo-control') : null;
                    
                    if (range) range.value = editedAngles[k];
                    if (input) input.value = editedAngles[k];
                    if (valueDisplay) valueDisplay.textContent = `${editedAngles[k]}°`;
                    
                    // Add visual feedback for successful move
                    if (servoControl) {
                        servoControl.style.background = 'rgba(67, 233, 123, 0.1)';
                        servoControl.style.border = '1px solid rgba(67, 233, 123, 0.3)';
                        setTimeout(() => {
                            servoControl.style.background = '';
                            servoControl.style.border = '';
                        }, 1000);
                    }
                });
                
                // Trigger camera update to show servo movement
                if (typeof updateCameraPosition === 'function') {
                    updateCameraPosition();
                }
                
                // Clear the queue
                for (const k of ids) delete editedAngles[k];
            } else {
                logMessage('❌ Batch move failed - check servo connections', 'error');
                
                // Add visual feedback for failed move
                ids.forEach(k => {
                    const range = document.getElementById(`servo${k}`);
                    const servoControl = range ? range.closest('.servo-control') : null;
                    if (servoControl) {
                        servoControl.style.background = 'rgba(231, 76, 60, 0.1)';
                        servoControl.style.border = '1px solid rgba(231, 76, 60, 0.3)';
                        setTimeout(() => {
                            servoControl.style.background = '';
                            servoControl.style.border = '';
                        }, 2000);
                    }
                });
            }
        }

        // Global servo configuration
        let servoConfigData = null;
        let lastConfigTimestamp = 0;
        
        async function fetchServoConfig() {
            try {
                const res = await fetch('/api/servo_config');
                const cfg = await res.json();
                
                if (!cfg || !cfg.servos) {
                    showConfigStatus('❌ No servo configuration found', 'error');
                    return;
                }
                
                // Check if config has changed
                if (cfg.timestamp !== lastConfigTimestamp) {
                    servoConfigData = cfg;
                    lastConfigTimestamp = cfg.timestamp;
                    generateServoControls(cfg);
                    updateServoControlTitle(cfg);
                    showConfigStatus(`✅ Loaded ${cfg.servos.length} servos from YAML config`, 'success');
                    logMessage(`🔧 Servo config loaded: ${cfg.servos.length} servos from YAML`, 'system');
                } else {
                    // Config hasn't changed, just update existing controls
                    applyServoLimits(cfg);
                }
                
                servoIdsFromConfig = cfg.servos.map(s => s.id);
                
            } catch(e) {
                console.error('Failed to load servo config', e);
                showConfigStatus('❌ Failed to load servo configuration', 'error');
                logMessage('❌ Failed to load servo configuration', 'error');
            }
        }
        
        function updateServoControlTitle(cfg) {
            const title = document.getElementById('servoControlTitle');
            if (title) {
                const configText = cfg.config_loaded ? 
                    `Servo Control (${cfg.servos.length} servos from YAML)` : 
                    'Servo Control (Default ranges)';
                title.innerHTML = `<i class="fas fa-robot"></i> ${configText}`;
            }
        }
        
        function showConfigStatus(message, type = 'info') {
            const statusDiv = document.getElementById('servoConfigStatus');
            const statusText = document.getElementById('configStatusText');
            
            if (statusDiv && statusText) {
                statusText.textContent = message;
                statusDiv.style.display = 'block';
                
                // Style based on type
                const colors = {
                    'success': 'rgba(67, 233, 123, 0.1)',
                    'error': 'rgba(231, 76, 60, 0.1)',
                    'warning': 'rgba(254, 225, 64, 0.1)',
                    'info': 'rgba(79, 172, 254, 0.1)'
                };
                statusDiv.style.background = colors[type] || colors['info'];
                
                // Auto-hide success messages
                if (type === 'success') {
                    setTimeout(() => {
                        statusDiv.style.display = 'none';
                    }, 3000);
                }
            }
        }
        
        function getServoIcon(name) {
            const iconMap = {
                'gripper': 'fas fa-grip-lines',
                'wrist': 'fas fa-hand-paper', 
                'elbow': 'fas fa-angle-double-right',
                'shoulder': 'fas fa-arrows-alt-v',
                'base': 'fas fa-sync'
            };
            return iconMap[name.toLowerCase()] || 'fas fa-cog';
        }
        
        function getServoColor(name) {
            const colorMap = {
                'gripper': '#fa709a',
                'wrist': '#4facfe',
                'elbow': '#4facfe', 
                'shoulder': '#4facfe',
                'base': '#4facfe'
            };
            return colorMap[name.toLowerCase()] || '#4facfe';
        }
        
        function generateServoControls(cfg) {
            const container = document.getElementById('servoControlsContainer');
            if (!container) return;
            
            // Clear existing controls
            container.innerHTML = '';
            
            // Set grid layout based on number of servos
            const servoCount = cfg.servos.length;
            if (servoCount <= 4) {
                container.style.gridTemplateColumns = 'repeat(2, 1fr)';
            } else {
                container.style.gridTemplateColumns = 'repeat(3, 1fr)';
            }
            
            cfg.servos.forEach(servo => {
                const servoDiv = document.createElement('div');
                servoDiv.className = 'servo-control';
                servoDiv.style.cssText = 'background: rgba(255,255,255,0.05); padding: 1rem; border-radius: 10px; transition: all 0.3s ease;';
                
                const icon = getServoIcon(servo.name);
                const color = getServoColor(servo.name);
                const displayName = servo.name.charAt(0).toUpperCase() + servo.name.slice(1);
                
                servoDiv.innerHTML = `
                    <label style="display: flex; align-items: center; margin-bottom: 0.5rem; font-weight: 600;">
                        <i class="${icon}" style="margin-right: 0.5rem; color: ${color};"></i> 
                        ${displayName} (ID ${servo.id})
                        <span class="servo-value" id="servo${servo.id}-value" style="margin-left: auto; color: #43e97b; font-weight: bold;">${servo.default_degrees}°</span>
                    </label>
                    <div style="margin-bottom: 0.5rem; font-size: 0.75rem; color: rgba(255,255,255,0.6);">
                        Range: ${servo.degrees_min}° to ${servo.degrees_max}°
                    </div>
                    <input type="range" id="servo${servo.id}" 
                           min="${servo.degrees_min}" max="${servo.degrees_max}" value="${servo.default_degrees}" 
                           oninput="updateServo(${servo.id}, this.value)"
                           style="width: 100%; margin-bottom: 0.5rem;">
                    <input class="servo-value" id="servo${servo.id}-input" 
                           type="number" step="1" min="${servo.degrees_min}" max="${servo.degrees_max}" value="${servo.default_degrees}" 
                           onkeydown="onServoInputKey(event, ${servo.id})"
                           style="width: 100%; padding: 0.3rem; border-radius: 5px; border: 1px solid rgba(255,255,255,0.2); background: rgba(255,255,255,0.1); color: white; text-align: center;">
                `;
                
                container.appendChild(servoDiv);
            });
            
            // If no servos, show a message
            if (cfg.servos.length === 0) {
                container.innerHTML = `
                    <div style="text-align: center; color: rgba(255,255,255,0.7); padding: 2rem; grid-column: 1 / -1;">
                        <i class="fas fa-exclamation-triangle" style="font-size: 1.5rem; margin-bottom: 0.5rem; color: #fee140;"></i><br>
                        No servos configured in YAML file
                    </div>
                `;
            }
        }
        
        function applyServoLimits(cfg) {
            cfg.servos.forEach(servo => {
                const slider = document.getElementById(`servo${servo.id}`);
                const input = document.getElementById(`servo${servo.id}-input`);
                
                if (slider) {
                    slider.min = servo.degrees_min;
                    slider.max = servo.degrees_max;
                    // Only reset value if it's outside the new range
                    if (parseInt(slider.value) < servo.degrees_min || parseInt(slider.value) > servo.degrees_max) {
                        slider.value = servo.default_degrees;
                    }
                }
                
                if (input) {
                    input.min = servo.degrees_min;
                    input.max = servo.degrees_max;
                    // Only reset value if it's outside the new range
                    if (parseInt(input.value) < servo.degrees_min || parseInt(input.value) > servo.degrees_max) {
                        input.value = servo.default_degrees;
                    }
                }
            });
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
                         <strong style="color: rgba(255,255,255,0.9);">Model Flip Controls:</strong>
                         <div style="margin-top: 0.5rem; display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.5rem;">
                             <div style="text-align: center;">
                                 <div style="font-size: 0.8rem; color: rgba(255,255,255,0.7); margin-bottom: 0.3rem;">X-Axis</div>
                                 <div style="display: flex; gap: 0.2rem;">
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('x', 1)" title="Normal X">
                                         <i class="fas fa-arrow-right"></i>
                                     </button>
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('x', -1)" title="Flip X">
                                         <i class="fas fa-arrow-left"></i>
                                     </button>
                                 </div>
                             </div>
                             <div style="text-align: center;">
                                 <div style="font-size: 0.8rem; color: rgba(255,255,255,0.7); margin-bottom: 0.3rem;">Y-Axis</div>
                                 <div style="display: flex; gap: 0.2rem;">
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('y', 1)" title="Normal Y">
                                         <i class="fas fa-arrow-up"></i>
                                     </button>
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('y', -1)" title="Flip Y">
                                         <i class="fas fa-arrow-down"></i>
                                     </button>
                                 </div>
                             </div>
                             <div style="text-align: center;">
                                 <div style="font-size: 0.8rem; color: rgba(255,255,255,0.7); margin-bottom: 0.3rem;">Z-Axis</div>
                                 <div style="display: flex; gap: 0.2rem;">
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('z', 1)" title="Normal Z">
                                         <i class="fas fa-level-up-alt"></i>
                                     </button>
                                     <button class="btn btn-warning" style="flex: 1; font-size: 0.8rem; padding: 0.3rem;" onclick="flipModel('z', -1)" title="Flip Z">
                                         <i class="fas fa-level-down-alt"></i>
                                     </button>
                                 </div>
                             </div>
                         </div>
                         <div style="margin-top: 0.5rem; text-align: center;">
                             <button class="btn btn-secondary" onclick="resetFlips()" style="font-size: 0.8rem; padding: 0.4rem 1rem;" title="Reset all orientations">
                                 <i class="fas fa-undo-alt"></i> Reset All
                             </button>
                         </div>
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
         let modelFlips = { x: 1, y: 1, z: 1 }; // Track flip states
         
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
                         
                         // Reset flip states for new model
                         modelFlips = { x: 1, y: 1, z: 1 };
                         
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
         
                 // Enhanced Model flip functionality with directional controls
        function flipModel(axis, direction = null) {
            if (!currentModel) {
                logMessage('❌ No model loaded to flip', 'warning');
                return;
            }
            
            // If direction is specified, use it; otherwise toggle
            if (direction !== null) {
                modelFlips[axis] = direction;
            } else {
                modelFlips[axis] *= -1;
            }
            
            // Apply the flip transformation with smooth animation
            const targetScale = {
                x: Math.abs(currentModel.scale.x) * modelFlips.x,
                y: Math.abs(currentModel.scale.y) * modelFlips.y,
                z: Math.abs(currentModel.scale.z) * modelFlips.z
            };
            
            // Animate the flip for better UX
            const startScale = { ...currentModel.scale };
            const duration = 300; // ms
            const startTime = Date.now();
            
            function animateFlip() {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const easeProgress = 1 - Math.pow(1 - progress, 3); // easeOutCubic
                
                if (axis === 'x') {
                    currentModel.scale.x = startScale.x + (targetScale.x - startScale.x) * easeProgress;
                } else if (axis === 'y') {
                    currentModel.scale.y = startScale.y + (targetScale.y - startScale.y) * easeProgress;
                } else if (axis === 'z') {
                    currentModel.scale.z = startScale.z + (targetScale.z - startScale.z) * easeProgress;
                }
                
                if (progress < 1) {
                    requestAnimationFrame(animateFlip);
                } else {
                    // Ensure exact final values
                    if (axis === 'x') currentModel.scale.x = targetScale.x;
                    else if (axis === 'y') currentModel.scale.y = targetScale.y;
                    else if (axis === 'z') currentModel.scale.z = targetScale.z;
                }
            }
            
            animateFlip();
            logMessage(`🔄 Model flipped along ${axis.toUpperCase()}-axis (${modelFlips[axis] > 0 ? 'normal' : 'flipped'})`, 'system');
        }
        
        // Reset all flips to normal
        function resetFlips() {
            if (!currentModel) {
                logMessage('❌ No model loaded to reset', 'warning');
                return;
            }
            
            modelFlips = { x: 1, y: 1, z: 1 };
            const baseScale = 2; // Adjust based on your model's default scale
            
            // Animate back to normal
            const targetScale = { x: baseScale, y: baseScale, z: baseScale };
            const startScale = { ...currentModel.scale };
            const duration = 500;
            const startTime = Date.now();
            
            function animateReset() {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const easeProgress = 1 - Math.pow(1 - progress, 3);
                
                currentModel.scale.x = startScale.x + (targetScale.x - startScale.x) * easeProgress;
                currentModel.scale.y = startScale.y + (targetScale.y - startScale.y) * easeProgress;
                currentModel.scale.z = startScale.z + (targetScale.z - startScale.z) * easeProgress;
                
                if (progress < 1) {
                    requestAnimationFrame(animateReset);
                } else {
                    currentModel.scale.set(baseScale, baseScale, baseScale);
                }
            }
            
            animateReset();
            logMessage('🔄 Model orientation reset to normal', 'system');
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