#!/usr/bin/env python3
# encoding: utf-8
"""
Jacobian Controller and Path Planning Demonstration
Shows various trajectory types and control capabilities
"""

import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin, cos
sys.path.append('/home/pi/ArmPi_mini/')

from kinematics.arm_move_ik import *
from kinematics.jacobian_controller import JacobianController
from common.ros_robot_controller_sdk import Board

def demonstrate_jacobian_capabilities():
    """Comprehensive demonstration of Jacobian-based control"""
    
    print("=== Jacobian Controller Demonstration ===")
    
    # Initialize hardware
    board = Board()
    AK = ArmIK()
    AK.board = board
    
    # Initialize Jacobian controller
    jc = JacobianController(AK)
    
    # Move to home position
    print("Moving to home position...")
    AK.setPitchRangeMoving((0, 10, 20), -90, -90, 90, 2000)
    time.sleep(3)
    
    # Update controller's current state
    jc.current_joint_angles = np.array([0.0, 90.0, 90.0, 0.0])
    
    try:
        # Demo 1: Point-to-point trajectories
        demonstrate_point_to_point(jc)
        
        # Demo 2: Circular trajectories  
        demonstrate_circular_motion(jc)
        
        # Demo 3: Smooth tracking
        demonstrate_smooth_tracking(jc)
        
        # Demo 4: Obstacle avoidance
        demonstrate_obstacle_avoidance(jc)
        
        # Demo 5: Velocity control
        demonstrate_velocity_control(jc)
        
    except KeyboardInterrupt:
        print("Demo interrupted by user")
    except Exception as e:
        print(f"Demo error: {e}")
    finally:
        # Return to home
        jc.stop_tracking()
        AK.setPitchRangeMoving((0, 10, 20), -90, -90, 90, 2000)
        print("Demo completed")

def demonstrate_point_to_point(jc):
    """Demonstrate different point-to-point trajectory types"""
    print("\n--- Point-to-Point Trajectory Demo ---")
    
    # Define waypoints
    waypoints = [
        [0, 15, 15, 0, -pi/2, 0],    # Forward
        [10, 10, 12, 0, -pi/2, 0],   # Right
        [-5, 12, 18, 0, -pi/2, 0],   # Left and up
        [0, 8, 10, 0, -pi/2, 0],     # Center low
    ]
    
    trajectory_types = ['linear', 'cubic', 'quintic', 'spline']
    
    for i, (start_pose, end_pose) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        traj_type = trajectory_types[i % len(trajectory_types)]
        print(f"Executing {traj_type} trajectory to waypoint {i+1}")
        
        # Generate trajectory
        trajectory = jc.smooth_point_to_point(start_pose, end_pose, 3.0, traj_type)
        
        # Execute trajectory
        jc.execute_trajectory(trajectory, 3.0)
        time.sleep(1)

def demonstrate_circular_motion(jc):
    """Demonstrate circular trajectory generation"""
    print("\n--- Circular Motion Demo ---")
    
    # Horizontal circle
    print("Executing horizontal circle...")
    center = [0, 12]
    radius = 5.0
    height = 15.0
    duration = 6.0
    
    circular_traj = jc.circular_trajectory(center, radius, height, duration, 'z')
    jc.execute_trajectory(circular_traj, duration)
    
    time.sleep(1)

def demonstrate_smooth_tracking(jc):
    """Demonstrate smooth tracking of a moving target"""
    print("\n--- Smooth Tracking Demo ---")
    
    # Create moving target function
    start_time = time.time()
    
    def moving_target():
        """Generate moving target trajectory"""
        elapsed = time.time() - start_time
        
        # Figure-8 pattern
        t = elapsed * 0.5  # Slow motion
        x = 8 * sin(t)
        y = 12 + 4 * sin(2 * t)
        z = 15 + 2 * cos(t)
        
        return np.array([x, y, z, 0, -pi/2, 0])
    
    print("Starting smooth tracking for 10 seconds...")
    jc.start_tracking(moving_target)
    time.sleep(10)
    jc.stop_tracking()
    
    print("Tracking completed")

def demonstrate_obstacle_avoidance(jc):
    """Demonstrate obstacle avoidance trajectory"""
    print("\n--- Obstacle Avoidance Demo ---")
    
    # Define obstacles (x, y, z, radius)
    obstacles = [
        (5, 10, 12, 3),   # Obstacle 1
        (-3, 8, 15, 2),   # Obstacle 2
    ]
    
    start_pose = [0, 15, 10, 0, -pi/2, 0]
    end_pose = [8, 5, 18, 0, -pi/2, 0]
    
    print("Planning trajectory with obstacle avoidance...")
    avoidance_traj = jc.obstacle_avoidance_trajectory(
        start_pose, end_pose, obstacles, clearance=3.0
    )
    
    print("Executing safe trajectory...")
    jc.execute_trajectory(avoidance_traj, 5.0)

def demonstrate_velocity_control(jc):
    """Demonstrate direct velocity control"""
    print("\n--- Velocity Control Demo ---")
    
    print("Executing sinusoidal velocity pattern...")
    
    start_time = time.time()
    duration = 8.0
    
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        t = elapsed
        
        # Generate sinusoidal velocity commands
        vx = 2.0 * sin(t)      # cm/s
        vy = 1.5 * cos(t * 1.5) # cm/s  
        vz = 0.8 * sin(t * 0.5) # cm/s
        wx, wy, wz = 0, 0, 0    # No rotational velocity
        
        target_velocity = np.array([vx, vy, vz, wx, wy, wz])
        
        # Compute and execute joint velocities
        joint_velocities = jc.velocity_control(target_velocity, jc.current_joint_angles)
        success = jc.execute_joint_velocities(joint_velocities, jc.dt)
        
        if not success:
            print("Velocity control failed")
            break
        
        time.sleep(jc.dt)
    
    print("Velocity control demo completed")

def analyze_workspace():
    """Analyze and visualize the robot workspace"""
    print("\n--- Workspace Analysis ---")
    
    # Initialize controller for analysis
    board = Board()
    AK = ArmIK()
    AK.board = board
    jc = JacobianController(AK)
    
    # Generate workspace points
    print("Analyzing reachable workspace...")
    
    # Sample joint space
    theta1_range = np.linspace(0, 180, 10)
    theta2_range = np.linspace(30, 150, 8)
    theta3_range = np.linspace(30, 150, 8)
    theta4_range = np.linspace(-90, 90, 7)
    
    reachable_points = []
    
    for θ1 in theta1_range:
        for θ2 in theta2_range:
            for θ3 in theta3_range:
                for θ4 in theta4_range:
                    joint_angles = [θ1, θ2, θ3, θ4]
                    pose = jc.forward_kinematics(joint_angles)
                    
                    # Check if pose is physically reasonable
                    x, y, z = pose[:3]
                    if 5 <= z <= 30 and np.sqrt(x**2 + y**2) <= 20:
                        reachable_points.append([x, y, z])
    
    reachable_points = np.array(reachable_points)
    
    print(f"Found {len(reachable_points)} reachable points")
    print(f"X range: {reachable_points[:, 0].min():.1f} to {reachable_points[:, 0].max():.1f} cm")
    print(f"Y range: {reachable_points[:, 1].min():.1f} to {reachable_points[:, 1].max():.1f} cm")
    print(f"Z range: {reachable_points[:, 2].min():.1f} to {reachable_points[:, 2].max():.1f} cm")
    
    # Save workspace data
    np.save('/tmp/armpi_workspace.npy', reachable_points)
    print("Workspace data saved to /tmp/armpi_workspace.npy")

def jacobian_analysis():
    """Analyze Jacobian properties at different configurations"""
    print("\n--- Jacobian Analysis ---")
    
    board = Board()
    AK = ArmIK()
    AK.board = board
    jc = JacobianController(AK)
    
    # Test configurations
    test_configs = [
        [0, 90, 90, 0],      # Home position
        [45, 45, 45, 0],     # Mid-range
        [90, 30, 150, -45],  # Near singularity
        [0, 120, 60, 30],    # Extended
    ]
    
    for i, config in enumerate(test_configs):
        print(f"\nConfiguration {i+1}: {config}")
        
        # Compute Jacobian
        J = jc.compute_jacobian(config)
        
        # Analyze properties
        manipulability = np.sqrt(np.linalg.det(J @ J.T))
        condition_number = np.linalg.cond(J)
        rank = np.linalg.matrix_rank(J)
        
        print(f"  Manipulability: {manipulability:.4f}")
        print(f"  Condition number: {condition_number:.2f}")
        print(f"  Rank: {rank}")
        
        if manipulability < 1e-3:
            print("  WARNING: Near singularity!")
        
        if condition_number > 100:
            print("  WARNING: Ill-conditioned!")

def performance_benchmark():
    """Benchmark the performance of different control methods"""
    print("\n--- Performance Benchmark ---")
    
    board = Board()
    AK = ArmIK()
    AK.board = board
    jc = JacobianController(AK)
    
    # Benchmark parameters
    n_iterations = 100
    test_pose = [10, 10, 15, 0, -pi/2, 0]
    
    # Benchmark Jacobian computation
    print("Benchmarking Jacobian computation...")
    start_time = time.time()
    for _ in range(n_iterations):
        J = jc.compute_jacobian([45, 90, 90, 0])
    jacobian_time = (time.time() - start_time) / n_iterations
    
    # Benchmark forward kinematics
    print("Benchmarking forward kinematics...")
    start_time = time.time()
    for _ in range(n_iterations):
        pose = jc.forward_kinematics([45, 90, 90, 0])
    fk_time = (time.time() - start_time) / n_iterations
    
    # Benchmark position control
    print("Benchmarking position control...")
    start_time = time.time()
    for _ in range(n_iterations):
        joint_vel = jc.position_control(test_pose, [45, 90, 90, 0])
    control_time = (time.time() - start_time) / n_iterations
    
    print(f"\nPerformance Results:")
    print(f"  Jacobian computation: {jacobian_time*1000:.2f} ms")
    print(f"  Forward kinematics:   {fk_time*1000:.2f} ms")
    print(f"  Position control:     {control_time*1000:.2f} ms")
    print(f"  Control frequency:    {1/control_time:.1f} Hz")

def main():
    """Main demonstration function"""
    print("ArmPi Mini - Advanced Jacobian Control Demonstration")
    print("=" * 60)
    
    while True:
        print("\nSelect demonstration:")
        print("1. Full capability demo")
        print("2. Workspace analysis") 
        print("3. Jacobian analysis")
        print("4. Performance benchmark")
        print("5. Exit")
        
        try:
            choice = input("Enter choice (1-5): ").strip()
            
            if choice == '1':
                demonstrate_jacobian_capabilities()
            elif choice == '2':
                analyze_workspace()
            elif choice == '3':
                jacobian_analysis()
            elif choice == '4':
                performance_benchmark()
            elif choice == '5':
                print("Exiting demonstration")
                break
            else:
                print("Invalid choice, please try again")
                
        except KeyboardInterrupt:
            print("\nExiting demonstration")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()