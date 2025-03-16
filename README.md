# ArmPi Mini - NVIDIA Isaac Sim Integration

This project enables communication between a physical HiWonder ArmPi Mini robot (4-DOF + manipulator + camera) and NVIDIA Isaac Sim's Franka robot simulation.

## Components

### ArmPi Server (`nvidia_isaac_Integration/armpi_server.py`)
- Runs on the Raspberry Pi 5 controlling the ArmPi Mini
- Listens for TCP connections on port 5000
- Receives joint positions and commands from Isaac Sim
- Controls the physical ArmPi robot using the Board SDK
- Captures and sends camera feed
- Sends back robot status (joint positions, lengths, manipulator status)

### Robot Control Client (on Isaac Sim machine)
- Connects to the ArmPi server
- Controls the Franka robot in simulation
- Sends Franka's joint positions to the ArmPi
- Receives and processes the ArmPi's status and camera feed
- Updates the simulation accordingly

## Setup

### Prerequisites
- Raspberry Pi 5 with ArmPi Mini robot
- NVIDIA Isaac Sim installed on a compatible machine
- Network connectivity between the two systems

### Installation
1. Clone this repository on the Raspberry Pi
2. Ensure all dependencies are installed
3. Run the ArmPi server on the Raspberry Pi
4. Run the Robot Control Client on the Isaac Sim machine

## Usage
1. Start the ArmPi server:
   ```
   python3 nvidia_isaac_Integration/armpi_server.py
   ```
2. Start the Robot Control Client on the Isaac Sim machine
3. The two systems will now communicate, with the physical robot mirroring the simulated Franka's movements

## License
[Your license information here] 