#!/usr/bin/python3
# coding=utf8
import os
import sys
sys.path.append('/home/pi/ArmPi_mini/')

# Add the paths to the common and kinematics modules
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'common_sdk'))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'kinematics_sdk'))

import time
import json
import logging
import threading
import numpy as np
import functions.running as running
import functions.lab_adjust as lab_adjust
import functions.color_sorting as color_sorting
import functions.color_detect as color_detect
import functions.color_tracking as color_tracking
import functions.color_palletizing as color_palletizing
from my_kinematics.arm_move_ik import *
from werkzeug.serving import run_simple
from werkzeug.wrappers import Request, Response
from jsonrpc2 import JsonRpc

# Hardware availability and safe Board access helpers
ROBOT_AVAILABLE = True
try:
    from common.ros_robot_controller_sdk import Board  # type: ignore
except Exception as e:
    print(f"Robot Board not available: {e}")
    Board = None  # type: ignore
    ROBOT_AVAILABLE = False

board = None  # Will be injected by main process

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def safe_board_call(method_name, *args, **kwargs):
    """Safely call a board method whether it's on the instance or class."""
    # Prefer instance method on injected board
    if board is not None and hasattr(board, method_name):
        return getattr(board, method_name)(*args, **kwargs)
    # Fall back to class/static if available
    if Board is not None and hasattr(Board, method_name):
        return getattr(Board, method_name)(*args, **kwargs)
    raise NameError("Board is not available")

# Compatibility dispatcher to preserve existing decorator and mapping usage
class _Dispatcher(dict):
    def __init__(self, rpc: JsonRpc) -> None:
        super().__init__()
        self._rpc = rpc

    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        # Mirror registrations into the jsonrpc2 router
        self._rpc[key] = value

    def add_method(self, func=None, name: str | None = None):
        def decorator(f):
            method_name = name or f.__name__
            self[method_name] = f
            return f
        return decorator(func) if func is not None else decorator

# Initialize JSON-RPC 2.0 router and a compatible dispatcher facade
_rpc = JsonRpc()
dispatcher = _Dispatcher(_rpc)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

HWSONAR = None
QUEUE = None
    
def set_board():
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


@dispatcher.add_method
def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

data = []
@dispatcher.add_method
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    print("SetPWMServo:", args)
    if not ROBOT_AVAILABLE:
        return (True, f"Demo: Servo moved with params {args}", 'SetPWMServo')
    arglen = len(args)
    try:
        servos = list(args[1:arglen:2])
        values = list(args[2:arglen:2])
        use_times_ms = args[0]
        data = []
        for (s, v) in zip(servos, values):
            if isinstance(v, (int, float)):
                if -90 <= v <= 90:
                    pulse_us = int((v - (-90)) * (2500 - 500) / (90 - (-90)) + 500)
                elif 0 <= v <= 180:
                    pulse_us = int((v - 0) * (2500 - 500) / (180 - 0) + 500)
                else:
                    pulse_us = int(v)
            else:
                pulse_us = 1500
            pulse_us = clamp(pulse_us, 500, 2500)
            data.append([s, pulse_us])
        safe_board_call('pwm_servo_set_position', use_times_ms/1000.0, data)
        data.clear()
    except Exception as e:
        print('error3:', e)
        ret = (False, __RPC_E03, 'SetPWMServo')
    return ret

@dispatcher.add_method
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
        for (s, p) in zip(servos, pulses):
            pulse_us = int(clamp(p, 500, 2500))
            safe_board_call('setBusServoPulse', s, pulse_us, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoPulse')
    return ret

@dispatcher.add_method
def SetBusServoDeviation(*args):
    ret = (True, (), 'SetBusServoDeviation')
    arglen = len(args)
    if arglen != 2:
        return (False, __RPC_E01, 'SetBusServoDeviation')
    try:
        servo = args[0]
        deviation = args[1]
        #Board.setBusServoDeviation(servo, deviation)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoDeviation')

@dispatcher.add_method
def GetBusServosDeviation(args):
    ret = (True, (), 'GetBusServosDeviation')
    data = []
    if args != "readDeviation":
        return (False, __RPC_E01, 'GetBusServosDeviation')
    try:
        for i in range(1, 7):
            dev = safe_board_call('getBusServoDeviation', i)
            if dev is None:
                dev = 999
            data.append(dev)
        ret = (True, data, 'GetBusServosDeviation')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosDeviation')
    return ret 

@dispatcher.add_method
def SaveBusServosDeviation(args):
    ret = (True, (), 'SaveBusServosDeviation')
    if args != "downloadDeviation":
        return (False, __RPC_E01, 'SaveBusServosDeviation')
    try:
        for i in range(1, 7):
            safe_board_call('saveBusServoDeviation', i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SaveBusServosDeviation')
    return ret 

@dispatcher.add_method
def UnloadBusServo(args):
    ret = (True, (), 'UnloadBusServo')
    if args != 'servoPowerDown':
        return (False, __RPC_E01, 'UnloadBusServo')
    try:
        for i in range(1, 7):
            safe_board_call('unloadBusServo', i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'UnloadBusServo')

@dispatcher.add_method
def GetBusServosPulse(args):
    ret = (True, (), 'GetBusServosPulse')
    data = []
    if args != 'angularReadback':
        return (False, __RPC_E01, 'GetBusServosPulse')
    try:
        for i in range(1, 7):
            pulse = safe_board_call('getBusServoPulse', i)
            if pulse is None:
                ret = (False, __RPC_E04, 'GetBusServosPulse')
                return ret
            else:
                data.append(pulse)
        ret = (True, data, 'GetBusServosPulse')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosPulse')
    return ret 
        
@dispatcher.add_method
def SetBrushMotor(*args, **kwargs):
    ret = (True, (), 'SetBrushMotor')
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

@dispatcher.add_method
def GetSonarDistance():
    global HWSONAR
    
    ret = (True, 0, 'GetSonarDistance')
    try:
        ret = (True, HWSONAR.getDistance(), 'GetSonarDistance')
    except:
        ret = (False, __RPC_E03, 'GetSonarDistance')
    return ret

@dispatcher.add_method
def GetBatteryVoltage():
    ret = (True, 0, 'GetBatteryVoltage')
    try:
        vb = None
        try:
            if board is not None and hasattr(board, 'getBattery'):
                vb = board.getBattery()
        except Exception:
            vb = None
        if vb is None and Board is not None and hasattr(Board, 'getBattery'):
            vb = Board.getBattery()
        ret = (True, vb, 'GetBatteryVoltage')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBatteryVoltage')
    return ret

@dispatcher.add_method
def SetSonarRGBMode(mode = 0):
    global HWSONAR
    
    HWSONAR.setRGBMode(mode)
    return (True, (mode,), 'SetSonarRGBMode')

@dispatcher.add_method
def SetSonarRGB(index, r, g, b):
    global HWSONAR
    
    if index == 0:
        HWSONAR.setRGB(1, (r, g, b))
        HWSONAR.setRGB(2, (r, g, b))
    else:
        HWSONAR.setRGB(index, (r, g, b))
    return (True, (r, g, b), 'SetSonarRGB')

@dispatcher.add_method
def SetSonarRGBBreathCycle(index, color, cycle):
    global HWSONAR
    
    HWSONAR.setBreathCycle(index, color, cycle)
    return (True, (index, color, cycle), 'SetSonarRGBBreathCycle')

@dispatcher.add_method
def SetSonarRGBStartSymphony():
    global HWSONAR
    
    HWSONAR.startSymphony()
    return (True, (), 'SetSonarRGBStartSymphony')

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

@dispatcher.add_method
def SetSonarDistanceThreshold(new_threshold = 30): 
    return runbymainth(Avoidance.setThreshold, (new_threshold,))

@dispatcher.add_method
def GetSonarDistanceThreshold():
    return runbymainth(Avoidance.getThreshold, ())

@dispatcher.add_method
def LoadFunc(new_func = 0):
    return runbymainth(running.loadFunc, (new_func, ))

@dispatcher.add_method
def UnloadFunc():
    return runbymainth(running.unloadFunc, ())

@dispatcher.add_method
def StartFunc():
    return runbymainth(running.startFunc, ())

@dispatcher.add_method
def StopFunc():
    return runbymainth(running.stopFunc, ())

@dispatcher.add_method
def FinishFunc():
    return runbymainth(running.finishFunc, ())

@dispatcher.add_method
def Heartbeat():
    return runbymainth(running.doHeartbeat, ())

@dispatcher.add_method
def GetRunningFunc():
    return runbymainth("GetRunningFunc", ())

@dispatcher.add_method
def ColorTracking(*target_color):
    return runbymainth(color_tracking.setTargetColor, target_color)

@dispatcher.add_method
def ColorSorting(*target_color):
    return runbymainth(color_sorting.setTargetColor, target_color)

@dispatcher.add_method
def ColorPalletizing(*target_color):
    return runbymainth(color_palletizing.setTargetColor, target_color)

# set color threshold
# parameter: color lab
# for example: [{'red': ((0, 0, 0), (255, 255, 255))}]
@dispatcher.add_method
def SetLABValue(*lab_value):
    #print(lab_value)
    return runbymainth(lab_adjust.setLABValue, lab_value)

# save color threshold
@dispatcher.add_method
def GetLABValue():
    return (True, lab_adjust.getLABValue()[1], 'GetLABValue')

# save color threshold
@dispatcher.add_method
def SaveLABValue(color=''):
    return runbymainth(lab_adjust.saveLABValue, (color, ))

@dispatcher.add_method
def HaveLABAdjust():
    return (True, True, 'HaveLABAdjust')

@Request.application
def application(request):
    dispatcher["echo"] = lambda s: s
    dispatcher["add"] = lambda a, b: a + b

    # Serve a simple help page for non-POST requests
    if request.method != 'POST':
        help_html = f"""
        <html><body>
        <h3>ArmPi Mini JSON-RPC 2.0</h3>
        <p>POST JSON-RPC requests to this endpoint.</p>
        <pre>curl -s -X POST http://{request.host}/ -H 'Content-Type: application/json' \
 -d '{{"jsonrpc":"2.0","method":"echo","params":["hi"],"id":1}}'</pre>
        </body></html>
        """
        return Response(help_html, mimetype='text/html')

    # Parse request payload and dispatch via jsonrpc2
    payload_bytes = request.data or b""
    if not payload_bytes:
        return Response("Empty request body; send JSON-RPC 2.0 payload.", status=400, mimetype='text/plain')

    try:
        if isinstance(payload_bytes, bytes):
            payload = json.loads(payload_bytes.decode("utf-8"))
        else:
            payload = json.loads(payload_bytes)
    except Exception:
        return Response("Invalid JSON payload.", status=400, mimetype='text/plain')

    response_obj = _rpc(payload)
    return Response(json.dumps(response_obj), mimetype='application/json')

def startRPCServer():
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    run_simple('', 9030, application)

if __name__ == '__main__':
    startRPCServer()
