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
from jsonrpc2 import JSONRPCResponseManager, dispatcher

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
    print("SetPWMServo:",args)
    arglen = len(args)
    try:
        servos = args[1:arglen:2]
        pulses = args[2:arglen:2]
        use_times = args[0]
        data = []
        
        dat = zip(servos, pulses)
        for (s, p) in dat:
            pulses = int(map(p,90,-90,500,2500))
            data.extend([[s,pulses]])
        board.pwm_servo_set_position(use_times/1000.0, data)
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
        dat = zip(servos, pulses)
        for (s, p) in dat:
            Board.setBusServoPulse(s, p, use_times)
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
            dev = Board.getBusServoDeviation(i)
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
            dev = Board.saveBusServoDeviation(i)
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
            Board.unloadBusServo(i)
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
            pulse = Board.getBusServoPulse(i)
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
        ret = (True, Board.getBattery(), 'GetBatteryVoltage')
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
    #print(request.data)
    response = JSONRPCResponseManager.handle(request.data, dispatcher)

    return Response(response.json, mimetype='application/json')

def startRPCServer():
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    run_simple('', 9030, application)

if __name__ == '__main__':
    startRPCServer()
