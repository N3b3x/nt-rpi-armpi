#!/usr/bin/env python3
# encoding: utf-8
"""
Advanced Jacobian-Based Control System for ArmPi Mini
Implements smooth tracking, velocity control, and path planning
"""

import numpy as np
import time
import threading
from math import sin, cos, pi, sqrt, atan2, radians, degrees
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline, UnivariateSpline
from scipy.optimize import minimize
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class JacobianController:
    """
    Advanced Jacobian-based controller for smooth tracking and path planning
    """
    
    def __init__(self, arm_ik_instance):
        """Initialize with existing ArmIK instance"""
        self.AK = arm_ik_instance
        self.board = arm_ik_instance.board
        
        # Physical parameters (from inversekinematics.py)
        self.l1 = 8.80  # Base height (cm)
        self.l2 = 5.80  # Shoulder to elbow (cm)
        self.l3 = 6.20  # Elbow to wrist (cm)
        self.l4 = 9.50  # Wrist to end-effector (cm)
        
        # Control parameters
        self.control_frequency = 50.0  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # Current state
        self.current_joint_angles = np.array([0.0, 90.0, 90.0, 0.0])  # [θ1, θ2, θ3, θ4] in degrees
        self.current_joint_velocities = np.array([0.0, 0.0, 0.0, 0.0])  # rad/s
        self.target_ee_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, z, rx, ry, rz]
        
        # Tracking state
        self.is_tracking = False
        self.tracking_thread = None
        self.tracking_active = False
        
        # Path planning
        self.current_trajectory = None
        self.trajectory_start_time = None
        
        # Control gains
        self.position_gain = 2.0
        self.orientation_gain = 1.0
        self.damping_factor = 0.1
        self.max_joint_velocity = radians(45)  # rad/s
        self.max_ee_velocity = 5.0  # cm/s
        
        # Singularity handling
        self.singularity_threshold = 1e-3
        self.damped_least_squares = True
        
        logger.info("Jacobian Controller initialized")

    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics: joint angles -> end-effector pose
        
        Args:
            joint_angles: [θ1, θ2, θ3, θ4] in degrees
            
        Returns:
            pose: [x, y, z, rx, ry, rz] - position (cm) and orientation (radians)
        """
        θ1, θ2, θ3, θ4 = np.radians(joint_angles)
        
        # Forward kinematics equations for 4-DOF arm
        # Position calculations
        c1, s1 = cos(θ1), sin(θ1)
        c2, s2 = cos(θ2), sin(θ2)
        c23, s23 = cos(θ2 + θ3), sin(θ2 + θ3)
        c234, s234 = cos(θ2 + θ3 + θ4), sin(θ2 + θ3 + θ4)
        
        # End-effector position
        x = (self.l2*c2 + self.l3*c23 + self.l4*c234) * c1
        y = (self.l2*c2 + self.l3*c23 + self.l4*c234) * s1
        z = self.l1 + self.l2*s2 + self.l3*s23 + self.l4*s234
        
        # End-effector orientation (simplified for 4-DOF)
        rx = 0.0  # No roll capability
        ry = -(θ2 + θ3 + θ4)  # Pitch angle
        rz = θ1  # Yaw angle
        
        return np.array([x, y, z, rx, ry, rz])

    def compute_jacobian(self, joint_angles):
        """
        Compute the Jacobian matrix J = ∂pose/∂q
        
        Args:
            joint_angles: [θ1, θ2, θ3, θ4] in degrees
            
        Returns:
            J: 6x4 Jacobian matrix [∂x/∂q, ∂y/∂q, ∂z/∂q, ∂rx/∂q, ∂ry/∂q, ∂rz/∂q]
        """
        θ1, θ2, θ3, θ4 = np.radians(joint_angles)
        
        c1, s1 = cos(θ1), sin(θ1)
        c2, s2 = cos(θ2), sin(θ2)
        c23, s23 = cos(θ2 + θ3), sin(θ2 + θ3)
        c234, s234 = cos(θ2 + θ3 + θ4), sin(θ2 + θ3 + θ4)
        
        # Partial derivatives for position
        # ∂x/∂θ1
        dx_dq1 = -(self.l2*c2 + self.l3*c23 + self.l4*c234) * s1
        # ∂x/∂θ2
        dx_dq2 = (-self.l2*s2 - self.l3*s23 - self.l4*s234) * c1
        # ∂x/∂θ3
        dx_dq3 = (-self.l3*s23 - self.l4*s234) * c1
        # ∂x/∂θ4
        dx_dq4 = -self.l4*s234 * c1
        
        # ∂y/∂θ1
        dy_dq1 = (self.l2*c2 + self.l3*c23 + self.l4*c234) * c1
        # ∂y/∂θ2
        dy_dq2 = (-self.l2*s2 - self.l3*s23 - self.l4*s234) * s1
        # ∂y/∂θ3
        dy_dq3 = (-self.l3*s23 - self.l4*s234) * s1
        # ∂y/∂θ4
        dy_dq4 = -self.l4*s234 * s1
        
        # ∂z/∂θ1
        dz_dq1 = 0.0
        # ∂z/∂θ2
        dz_dq2 = self.l2*c2 + self.l3*c23 + self.l4*c234
        # ∂z/∂θ3
        dz_dq3 = self.l3*c23 + self.l4*c234
        # ∂z/∂θ4
        dz_dq4 = self.l4*c234
        
        # Orientation derivatives (simplified for 4-DOF)
        # ∂rx/∂q (no roll capability)
        drx_dq = [0.0, 0.0, 0.0, 0.0]
        
        # ∂ry/∂q (pitch)
        dry_dq = [0.0, -1.0, -1.0, -1.0]
        
        # ∂rz/∂q (yaw)
        drz_dq = [1.0, 0.0, 0.0, 0.0]
        
        # Assemble Jacobian matrix
        J = np.array([
            [dx_dq1, dx_dq2, dx_dq3, dx_dq4],
            [dy_dq1, dy_dq2, dy_dq3, dy_dq4],
            [dz_dq1, dz_dq2, dz_dq3, dz_dq4],
            drx_dq,
            dry_dq,
            drz_dq
        ])
        
        return J

    def damped_least_squares_inverse(self, J, damping=None):
        """
        Compute damped least squares pseudo-inverse for singularity-robust control
        
        Args:
            J: Jacobian matrix
            damping: Damping factor (if None, use adaptive damping)
            
        Returns:
            J_inv: Pseudo-inverse matrix
        """
        if damping is None:
            # Adaptive damping based on manipulability
            manipulability = sqrt(np.linalg.det(J @ J.T))
            if manipulability < self.singularity_threshold:
                damping = self.damping_factor * (1.0 - manipulability / self.singularity_threshold)**2
            else:
                damping = 1e-6
        
        # Damped least squares: J† = J^T(JJ^T + λ²I)^(-1)
        JJT = J @ J.T
        damping_matrix = damping**2 * np.eye(JJT.shape[0])
        
        try:
            J_inv = J.T @ np.linalg.inv(JJT + damping_matrix)
        except np.linalg.LinAlgError:
            logger.warning("Singular matrix encountered, using high damping")
            J_inv = J.T @ np.linalg.inv(JJT + 0.1 * np.eye(JJT.shape[0]))
        
        return J_inv

    def velocity_control(self, target_ee_velocity, joint_angles):
        """
        Compute joint velocities for desired end-effector velocity
        
        Args:
            target_ee_velocity: [vx, vy, vz, wx, wy, wz] desired EE velocity
            joint_angles: Current joint angles in degrees
            
        Returns:
            joint_velocities: [ω1, ω2, ω3, ω4] in rad/s
        """
        J = self.compute_jacobian(joint_angles)
        
        if self.damped_least_squares:
            J_inv = self.damped_least_squares_inverse(J)
        else:
            J_inv = np.linalg.pinv(J)
        
        # Compute joint velocities: q̇ = J†ẋ
        joint_velocities = J_inv @ target_ee_velocity
        
        # Apply velocity limits
        joint_velocities = np.clip(joint_velocities, 
                                 -self.max_joint_velocity, 
                                  self.max_joint_velocity)
        
        return joint_velocities

    def position_control(self, target_pose, current_joint_angles):
        """
        Position-based control using Jacobian transpose or pseudo-inverse
        
        Args:
            target_pose: [x, y, z, rx, ry, rz] target end-effector pose
            current_joint_angles: Current joint angles in degrees
            
        Returns:
            joint_velocities: [ω1, ω2, ω3, ω4] in rad/s
        """
        # Current end-effector pose
        current_pose = self.forward_kinematics(current_joint_angles)
        
        # Pose error
        pose_error = target_pose - current_pose
        
        # Separate position and orientation errors
        position_error = pose_error[:3]
        orientation_error = pose_error[3:]
        
        # Scale errors by gains
        position_error *= self.position_gain
        orientation_error *= self.orientation_gain
        
        # Limit maximum end-effector velocity
        position_velocity_magnitude = np.linalg.norm(position_error)
        if position_velocity_magnitude > self.max_ee_velocity:
            position_error = position_error * (self.max_ee_velocity / position_velocity_magnitude)
        
        # Combine into desired end-effector velocity
        target_ee_velocity = np.concatenate([position_error, orientation_error])
        
        # Convert to joint velocities
        return self.velocity_control(target_ee_velocity, current_joint_angles)

    def execute_joint_velocities(self, joint_velocities, dt):
        """
        Execute joint velocities by integrating to positions and sending servo commands
        
        Args:
            joint_velocities: [ω1, ω2, ω3, ω4] in rad/s
            dt: Time step in seconds
        """
        # Integrate velocities to get new joint angles
        delta_angles = np.degrees(joint_velocities * dt)
        new_joint_angles = self.current_joint_angles + delta_angles
        
        # Apply joint limits (based on servo ranges)
        new_joint_angles = np.clip(new_joint_angles, [0, 0, 0, -180], [180, 180, 180, 180])
        
        # Convert to servo commands using existing ArmIK transform
        try:
            θ1, θ2, θ3, θ4 = new_joint_angles
            servos = self.AK.transformAngelAdaptArm(θ2, θ3, θ4, θ1)  # Note: different order in ArmIK
            
            if servos:
                # Execute with short movement time for smooth motion
                movement_time = max(0.02, dt)  # Minimum 20ms
                self.AK.servosMove((servos["servo3"], servos["servo4"], 
                                  servos["servo5"], servos["servo6"]), 
                                 int(movement_time * 1000))
                
                # Update current state
                self.current_joint_angles = new_joint_angles
                self.current_joint_velocities = joint_velocities
                
                return True
        except Exception as e:
            logger.error(f"Failed to execute joint velocities: {e}")
            return False
        
        return False

    def start_tracking(self, target_pose_callback):
        """
        Start continuous tracking of a moving target
        
        Args:
            target_pose_callback: Function that returns current target pose [x, y, z, rx, ry, rz]
        """
        if self.is_tracking:
            logger.warning("Tracking already active")
            return
        
        self.is_tracking = True
        self.tracking_active = True
        
        def tracking_loop():
            logger.info("Starting Jacobian tracking loop")
            
            while self.tracking_active:
                start_time = time.time()
                
                try:
                    # Get current target pose
                    target_pose = target_pose_callback()
                    
                    if target_pose is not None:
                        # Compute control action
                        joint_velocities = self.position_control(target_pose, self.current_joint_angles)
                        
                        # Execute motion
                        success = self.execute_joint_velocities(joint_velocities, self.dt)
                        
                        if not success:
                            logger.warning("Failed to execute tracking command")
                    
                except Exception as e:
                    logger.error(f"Tracking error: {e}")
                
                # Maintain control frequency
                elapsed = time.time() - start_time
                sleep_time = max(0, self.dt - elapsed)
                time.sleep(sleep_time)
            
            logger.info("Tracking loop ended")
        
        self.tracking_thread = threading.Thread(target=tracking_loop, daemon=True)
        self.tracking_thread.start()

    def stop_tracking(self):
        """Stop continuous tracking"""
        if self.is_tracking:
            self.tracking_active = False
            self.is_tracking = False
            if self.tracking_thread:
                self.tracking_thread.join(timeout=1.0)
            logger.info("Tracking stopped")

    def smooth_point_to_point(self, start_pose, end_pose, duration, trajectory_type='quintic'):
        """
        Generate smooth point-to-point trajectory
        
        Args:
            start_pose: Starting pose [x, y, z, rx, ry, rz]
            end_pose: Ending pose [x, y, z, rx, ry, rz]
            duration: Trajectory duration in seconds
            trajectory_type: 'linear', 'cubic', 'quintic', or 'spline'
            
        Returns:
            trajectory_func: Function that takes time t and returns pose
        """
        start_pose = np.array(start_pose)
        end_pose = np.array(end_pose)
        
        if trajectory_type == 'linear':
            def trajectory(t):
                alpha = np.clip(t / duration, 0, 1)
                return start_pose + alpha * (end_pose - start_pose)
                
        elif trajectory_type == 'cubic':
            def trajectory(t):
                alpha = np.clip(t / duration, 0, 1)
                # Cubic polynomial: 3α² - 2α³
                s = 3*alpha**2 - 2*alpha**3
                return start_pose + s * (end_pose - start_pose)
                
        elif trajectory_type == 'quintic':
            def trajectory(t):
                alpha = np.clip(t / duration, 0, 1)
                # Quintic polynomial: 10α³ - 15α⁴ + 6α⁵
                s = 10*alpha**3 - 15*alpha**4 + 6*alpha**5
                return start_pose + s * (end_pose - start_pose)
                
        elif trajectory_type == 'spline':
            # Use scipy spline interpolation
            time_points = np.array([0, duration/3, 2*duration/3, duration])
            pose_points = np.array([start_pose, 
                                  start_pose + 0.3*(end_pose - start_pose),
                                  start_pose + 0.7*(end_pose - start_pose),
                                  end_pose])
            
            splines = []
            for i in range(6):  # 6 DOF
                splines.append(CubicSpline(time_points, pose_points[:, i]))
            
            def trajectory(t):
                t = np.clip(t, 0, duration)
                return np.array([spline(t) for spline in splines])
        
        return trajectory

    def execute_trajectory(self, trajectory_func, duration):
        """
        Execute a planned trajectory
        
        Args:
            trajectory_func: Function that takes time and returns pose
            duration: Total trajectory duration
        """
        logger.info(f"Executing trajectory for {duration} seconds")
        
        start_time = time.time()
        
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            
            if elapsed >= duration:
                break
            
            # Get target pose at current time
            target_pose = trajectory_func(elapsed)
            
            # Compute and execute control
            joint_velocities = self.position_control(target_pose, self.current_joint_angles)
            success = self.execute_joint_velocities(joint_velocities, self.dt)
            
            if not success:
                logger.warning("Trajectory execution failed")
                break
            
            # Maintain control frequency
            time.sleep(max(0, self.dt - (time.time() - current_time)))
        
        logger.info("Trajectory execution completed")

    def circular_trajectory(self, center, radius, height, duration, axis='z'):
        """
        Generate circular trajectory
        
        Args:
            center: [x, y] center of circle
            radius: Circle radius in cm
            height: Z height for circle
            duration: Duration for one complete circle
            axis: Rotation axis ('z' for horizontal circle)
            
        Returns:
            trajectory_func: Function that takes time t and returns pose
        """
        def trajectory(t):
            angle = 2 * pi * (t % duration) / duration
            
            if axis == 'z':
                x = center[0] + radius * cos(angle)
                y = center[1] + radius * sin(angle)
                z = height
                rx, ry, rz = 0, 0, 0
            
            return np.array([x, y, z, rx, ry, rz])
        
        return trajectory

    def obstacle_avoidance_trajectory(self, start_pose, end_pose, obstacles, clearance=2.0):
        """
        Generate trajectory with obstacle avoidance using potential fields
        
        Args:
            start_pose: Starting pose
            end_pose: Ending pose  
            obstacles: List of obstacles [(x, y, z, radius), ...]
            clearance: Minimum clearance distance
            
        Returns:
            trajectory_func: Safe trajectory function
        """
        def potential_field(position):
            """Compute attractive + repulsive potential"""
            pos = np.array(position[:3])
            goal = np.array(end_pose[:3])
            
            # Attractive potential (toward goal)
            attractive = 0.5 * np.linalg.norm(pos - goal)**2
            
            # Repulsive potential (away from obstacles)
            repulsive = 0.0
            for obs_x, obs_y, obs_z, obs_r in obstacles:
                obs_pos = np.array([obs_x, obs_y, obs_z])
                dist = np.linalg.norm(pos - obs_pos)
                
                if dist < obs_r + clearance:
                    repulsive += 0.5 * (1/(dist - obs_r) - 1/clearance)**2
            
            return attractive + repulsive
        
        # Use optimization to find way points
        n_waypoints = 5
        waypoints = []
        
        for i in range(n_waypoints + 1):
            alpha = i / n_waypoints
            initial_guess = start_pose[:3] + alpha * (np.array(end_pose[:3]) - np.array(start_pose[:3]))
            
            result = minimize(potential_field, initial_guess, method='BFGS')
            waypoints.append(result.x)
        
        # Create spline through waypoints
        time_points = np.linspace(0, 1, len(waypoints))
        waypoints = np.array(waypoints)
        
        splines = []
        for i in range(3):  # x, y, z
            splines.append(CubicSpline(time_points, waypoints[:, i]))
        
        def trajectory(t):
            t_norm = np.clip(t / 3.0, 0, 1)  # 3 second trajectory
            pos = np.array([spline(t_norm) for spline in splines])
            # Maintain orientation from start to end
            alpha = t_norm
            orient = np.array(start_pose[3:]) + alpha * (np.array(end_pose[3:]) - np.array(start_pose[3:]))
            return np.concatenate([pos, orient])
        
        return trajectory

    def get_current_pose(self):
        """Get current end-effector pose"""
        return self.forward_kinematics(self.current_joint_angles)

    def set_control_gains(self, position_gain=None, orientation_gain=None, damping_factor=None):
        """Update control gains"""
        if position_gain is not None:
            self.position_gain = position_gain
        if orientation_gain is not None:
            self.orientation_gain = orientation_gain
        if damping_factor is not None:
            self.damping_factor = damping_factor
        
        logger.info(f"Control gains updated: pos={self.position_gain}, "
                   f"orient={self.orientation_gain}, damping={self.damping_factor}")