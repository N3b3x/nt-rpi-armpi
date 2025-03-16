#!/usr/bin/env python3
import socket
import json
import struct
import cv2
import numpy as np
from typing import List, Tuple
import time
from isaacsim import SimulationApp
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
import sys
import carb
import threading

class FrankaArmPiClient:
    def __init__(self, host='raspberrypi.local', port=5000):
        # TCP Client setup
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
        # Initialize Isaac Sim
        self.simulation_app = SimulationApp({"headless": False})
        
        # Setup the simulation world
        self._setup_simulation()
        
        # Connect to ArmPi server
        self._connect_to_server()

    def _setup_simulation(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            self.simulation_app.close()
            sys.exit()

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Set camera view
        set_camera_view(
            eye=[5.0, 0.0, 1.5], 
            target=[0.00, 0.00, 1.00], 
            camera_prim_path="/OmniverseKit_Persp"
        )

        # Add Franka
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Arm")
        self.franka = Articulation(prim_paths_expr="/World/Arm", name="franka_arm")
        
        # Set initial pose
        self.franka.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units())
        
        # Initialize the world
        self.world.reset()

    def _connect_to_server(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to ArmPi server at {self.host}:{self.port}")
            
            # Start receive thread
            self.receive_thread = threading.Thread(target=self._receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
        except Exception as e:
            print(f"Failed to connect to ArmPi server: {e}")
            self.connected = False

    def _receive_data(self):
        while self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    break
                    
                # Parse received data
                message = json.loads(data.decode())
                self._handle_armpi_message(message)
                
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
        
        self.connected = False

    def _handle_armpi_message(self, message):
        """Handle messages received from ArmPi"""
        if 'type' not in message:
            return
            
        if message['type'] == 'joint_state':
            # Process joint state updates from ArmPi
            print(f"Received ArmPi joint states: {message['data']}")
            
        elif message['type'] == 'status':
            # Process status updates
            print(f"ArmPi status: {message['data']}")

    def send_joint_positions(self, positions):
        """Send joint positions to ArmPi"""
        if not self.connected:
            print("Not connected to ArmPi server")
            return
            
        message = {
            'type': 'joint_positions',
            'data': positions.tolist()
        }
        
        try:
            self.socket.send(json.dumps(message).encode())
        except Exception as e:
            print(f"Error sending joint positions: {e}")
            self.connected = False

    def run_simulation(self):
        """Main simulation loop"""
        try:
            while True:
                # Get Franka's current joint positions
                franka_positions = self.franka.get_joint_positions()[0]
                
                # Send to ArmPi if connected
                if self.connected:
                    self.send_joint_positions(franka_positions)
                
                # Step the simulation
                self.world.step(render=True)
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        if self.socket:
            self.socket.close()
        self.simulation_app.close()

if __name__ == "__main__":
    # Create and run the client
    client = FrankaArmPiClient(host='raspberrypi.local', port=5000)
    client.run_simulation() 