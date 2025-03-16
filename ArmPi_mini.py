#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import queue
sys.path.append('/home/pi/ArmPi_mini/')
import Camera
import logging
import threading
import rpc_server
import mjpg_server
import numpy as np
import functions.running as running
from kinematics.arm_move_ik import *
from common.ros_robot_controller_sdk import Board

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# instantiate inverse kinematics library
AK = ArmIK()
board = Board()
board.enable_reception()    

QUEUE_RPC = queue.Queue(10)

def startMiniPi():
    global HWEXT, HWSONIC
    
    
    AK.board = board
    rpc_server.board = board
    rpc_server.AK = AK
    rpc_server.set_board()    

    rpc_server.QUEUE = QUEUE_RPC

    threading.Thread(target=rpc_server.startRPCServer,
                     daemon=True).start()  # rpc server
    threading.Thread(target=mjpg_server.startMjpgServer,
                     daemon=True).start()  # mjpq steam server
    
    loading_picture = cv2.imread('/home/pi/ArmPi_mini/CameraCalibration/loading.jpg')
    cam = Camera.Camera()  # camera read
    cam.camera_open()
    running.cam = cam

    while True:
        time.sleep(0.03)
        # execute the RPC command to be executed in the current thread
        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                event, params, *_ = ret
                ret[2] = req(params)  # execute PRC command
                event.set()
            except:
                break

        # execute function game program
        try:
            if running.RunningFunc > 0 and running.RunningFunc <= 9:
                if cam.frame is not None:
                    frame = cam.frame.copy()
                    img = running.CurrentEXE().run(frame)
                    if running.RunningFunc == 9:
                        mjpg_server.img_show = np.vstack((img, frame))
                    else:                       
                        mjpg_server.img_show = img
                else:
                    mjpg_server.img_show = loading_picture
            else:
                mjpg_server.img_show = cam.frame
                #cam.frame = None
        except KeyboardInterrupt:
            print('RunningFunc1', running.RunningFunc)
            break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startMiniPi()
