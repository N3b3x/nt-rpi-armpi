#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import queue
sys.path.append('/home/pi/ArmPi_mini/')

# Add the paths to the common and kinematics modules
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'common_sdk'))
sys.path.append(os.path.join(current_dir, 'armpi_mini_sdk', 'kinematics_sdk'))

import Camera
import logging
import threading
import rpc_server
import mjpg_server
import web_server
import numpy as np

# Initialize ROBOT_AVAILABLE flag
ROBOT_AVAILABLE = True

# Try to import robot-specific modules
try:
    import functions.running as running
    from my_kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    print("Robot modules loaded successfully")
except ImportError as e:
    print(f"Robot modules not available: {e}")
    ROBOT_AVAILABLE = False
    # Create dummy classes for demo mode
    class DummyBoard:
        def enable_reception(self): pass
        def set_buzzer(self, *args): pass
    class DummyAK:
        def __init__(self): self.board = None
    ArmIK = lambda: DummyAK()
    Board = DummyBoard

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# instantiate inverse kinematics library
AK = ArmIK()

# Initialize Board with try-except
board = None
try:
    board = Board()
    board.enable_reception()
    print("Board initialized successfully")
except Exception as e:
    print(f"Board initialization failed: {e}")
    ROBOT_AVAILABLE = False
    board = DummyBoard()

QUEUE_RPC = queue.Queue(10)

def create_default_loading_screen():
    """Create a simple default loading screen"""
    # Create a 640x480 image with gradient background
    height, width = 480, 640
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Create gradient background (purple to blue)
    for y in range(height):
        ratio = y / height
        b = int(255 * (0.4 + 0.3 * ratio))  # Blue
        g = int(255 * (0.1 + 0.2 * ratio))  # Green  
        r = int(255 * (0.8 - 0.3 * ratio))  # Red
        img[y, :] = [b, g, r]
    
    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    title = "ArmPi Mini Robot"
    subtitle = "Initializing..."
    
    # Calculate text size and position
    title_scale = 1.5
    subtitle_scale = 1.0
    title_thickness = 2
    subtitle_thickness = 2
    
    (title_w, title_h), _ = cv2.getTextSize(title, font, title_scale, title_thickness)
    (subtitle_w, subtitle_h), _ = cv2.getTextSize(subtitle, font, subtitle_scale, subtitle_thickness)
    
    title_x = (width - title_w) // 2
    title_y = height // 2 - 20
    subtitle_x = (width - subtitle_w) // 2
    subtitle_y = title_y + title_h + 30
    
    # Add text shadow
    cv2.putText(img, title, (title_x + 2, title_y + 2), font, title_scale, (50, 50, 50), title_thickness)
    cv2.putText(img, subtitle, (subtitle_x + 2, subtitle_y + 2), font, subtitle_scale, (50, 50, 50), subtitle_thickness)
    
    # Add main text
    cv2.putText(img, title, (title_x, title_y), font, title_scale, (255, 255, 255), title_thickness)
    cv2.putText(img, subtitle, (subtitle_x, subtitle_y), font, subtitle_scale, (200, 200, 255), subtitle_thickness)
    
    # Add simple robot icon (circles and rectangles)
    robot_x, robot_y = width // 2, height // 2 - 100
    cv2.circle(img, (robot_x, robot_y), 30, (200, 200, 255), -1)  # Head
    cv2.circle(img, (robot_x, robot_y), 30, (255, 255, 255), 2)   # Head border
    cv2.rectangle(img, (robot_x - 20, robot_y + 20), (robot_x + 20, robot_y + 60), (180, 180, 255), -1)  # Body
    cv2.rectangle(img, (robot_x - 20, robot_y + 20), (robot_x + 20, robot_y + 60), (255, 255, 255), 2)   # Body border
    
    # Eyes
    cv2.circle(img, (robot_x - 8, robot_y - 5), 3, (100, 100, 255), -1)
    cv2.circle(img, (robot_x + 8, robot_y - 5), 3, (100, 100, 255), -1)
    
    return img

def startMiniPi():
    global HWEXT, HWSONIC, cam, ROBOT_AVAILABLE
    
    if ROBOT_AVAILABLE:
        AK.board = board
        rpc_server.board = board
        rpc_server.AK = AK
        rpc_server.set_board()
    else:
        print("Running in demo mode - hardware features disabled")

    rpc_server.QUEUE = QUEUE_RPC

    threading.Thread(target=rpc_server.startRPCServer,
                     daemon=True).start()  # rpc server
    threading.Thread(target=mjpg_server.startMjpgServer,
                     daemon=True).start()  # mjpeg steam server
    
    # Start the modern web server with all integrated functionality
    print("Starting integrated web server on port 8000...")
    print("   Access the modern robot control interface at:")
    print("   http://localhost:8000")
    try:
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        print(f"   http://{local_ip}:8000")
    except:
        pass
    
    # Initialize Camera with try-except
    cam = None
    try:
        cam = Camera.Camera()  # camera read
        cam.camera_open()
        if ROBOT_AVAILABLE:
            running.cam = cam
        print("Camera initialized successfully")
    except Exception as e:
        print(f"Camera initialization failed: {e}")
        ROBOT_AVAILABLE = False
        cam = None
    
    # Start web server with ROBOT_AVAILABLE flag
    threading.Thread(target=web_server.startWebServer,
                     args=(cam, board, AK, QUEUE_RPC, ROBOT_AVAILABLE),
                     daemon=True).start()  # integrated web server
    
    # Use project-relative loading image to avoid hardcoded absolute paths
    loading_picture_path = os.path.join(current_dir, 'CameraCalibration', 'loading.jpg')
    loading_picture = cv2.imread(loading_picture_path)
    
    # Create a simple loading image if the file doesn't exist
    if loading_picture is None:
        print("Creating default loading screen...")
        loading_picture = create_default_loading_screen()
        # Try to save it for future use
        try:
            cv2.imwrite(loading_picture_path, loading_picture)
        except:
            pass

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
            if ROBOT_AVAILABLE and running.RunningFunc > 0 and running.RunningFunc <= 9:
                if cam and cam.frame is not None:
                    frame = cam.frame.copy()
                    img = running.CurrentEXE().run(frame)
                    if running.RunningFunc == 9:
                        mjpg_server.img_show = np.vstack((img, frame))
                    else:                       
                        mjpg_server.img_show = img
                else:
                    mjpg_server.img_show = loading_picture
            else:
                if cam and cam.frame is not None:
                    mjpg_server.img_show = cam.frame
                else:
                    mjpg_server.img_show = loading_picture
        except KeyboardInterrupt:
            if ROBOT_AVAILABLE:
                print('RunningFunc1', running.RunningFunc)
            break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startMiniPi()
