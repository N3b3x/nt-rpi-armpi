#!/usr/bin/env python3
import socket
import json
import struct
import cv2
import numpy as np
from typing import List, Tuple
import time
import sys
import os

# Add ArmPi_mini path to system path
sys.path.append('/home/pi/ArmPi_mini/')
from common.ros_robot_controller_sdk import Board
from kinematics.arm_move_ik import ArmIK
import Camera

class ArmPiServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 5000):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        
        # Initialize robot control
        self.board = Board()
        self.board.enable_reception()
        self.ak = ArmIK()
        self.ak.board = self.board
        
        # Initialize camera
        self.camera = Camera.Camera()
        self.camera.camera_open()
        
        # Robot state
        self.joint_positions = [90, 90, 90, 90]  # Default servo angles
        self.manipulator_status = 0  # 0: closed, 1: open
        self.joint_lengths = [9.5, 10.5, 10.0, 16.0]  # Arm segment lengths in cm
        
    def send_image(self, client_socket: socket.socket):
        """Send camera frame over socket"""
        if self.camera.frame is not None:
            frame = self.camera.frame.copy()
            # Encode image as JPEG
            _, img_encoded = cv2.imencode('.jpg', frame)
            # Send image size first
            size = len(img_encoded)
            client_socket.send(struct.pack('!I', size))
            # Send image data
            client_socket.send(img_encoded.tobytes())
    
    def send_robot_status(self, client_socket: socket.socket):
        """Send robot status (joint positions, lengths, manipulator status)"""
        # Get current servo angles
        self.joint_positions = [
            self.board.get_servo_angle(1),  # Base
            self.board.get_servo_angle(2),  # Shoulder
            self.board.get_servo_angle(3),  # Elbow
            self.board.get_servo_angle(4)   # Wrist
        ]
        
        status = {
            'joint_positions': self.joint_positions,
            'joint_lengths': self.joint_lengths,
            'manipulator_status': self.manipulator_status
        }
        data = json.dumps(status).encode()
        client_socket.send(struct.pack('!I', len(data)))
        client_socket.send(data)
    
    def receive_command(self, client_socket: socket.socket) -> dict:
        """Receive command from client"""
        size_data = client_socket.recv(4)
        if not size_data:
            return None
        size = struct.unpack('!I', size_data)[0]
        data = client_socket.recv(size)
        return json.loads(data.decode())
    
    def execute_command(self, command: dict):
        """Execute received command"""
        if 'joint_positions' in command:
            angles = command['joint_positions']
            # Set servo angles
            for i, angle in enumerate(angles, start=1):
                self.board.set_servo_angle(i, int(angle))
                
        if 'manipulator_command' in command:
            self.manipulator_status = command['manipulator_command']
            # Set gripper state (servo 5)
            angle = 180 if self.manipulator_status == 1 else 0
            self.board.set_servo_angle(5, angle)
    
    def run(self):
        print(f"ArmPi Server listening on {self.host}:{self.port}")
        while True:
            client_socket, addr = self.socket.accept()
            print(f"Connected to {addr}")
            
            try:
                while True:
                    command = self.receive_command(client_socket)
                    if command is None:
                        break
                    
                    # Execute command
                    self.execute_command(command)
                    
                    # Send robot status
                    self.send_robot_status(client_socket)
                    
                    # Send camera frame
                    self.send_image(client_socket)
                    
            except Exception as e:
                print(f"Error: {e}")
            finally:
                client_socket.close()
                print(f"Disconnected from {addr}")

if __name__ == "__main__":
    server = ArmPiServer()
    server.run() 