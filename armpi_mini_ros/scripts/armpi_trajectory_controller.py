#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import os

# Add the ArmPi SDK to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../armpi_mini_sdk'))
from kinematics_sdk.my_kinematics.arm_move_ik import ArmIK

class ArmPiTrajectoryController:
    """
    ROS node that receives joint trajectory commands and executes them on the ArmPi Mini.
    This runs on the Raspberry Pi and interfaces with the physical hardware.
    """
    
    def __init__(self):
        rospy.init_node('armpi_trajectory_controller', anonymous=True)
        
        # Initialize ArmPi hardware interface
        self.arm_ik = ArmIK()
        
        # Joint names (must match URDF)
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
        
        # Subscribers
        self.trajectory_sub = rospy.Subscriber(
            '/arm_controller/follow_joint_trajectory/goal',
            JointTrajectory,
            self.trajectory_callback
        )
        
        # Publishers
        self.joint_state_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10
        )
        
        # Execution parameters
        self.execution_rate = rospy.get_param('~execution_rate', 50.0)  # Hz
        self.rate = rospy.Rate(self.execution_rate)
        
        # Current joint positions
        self.current_joint_positions = [0.0] * len(self.joint_names)
        
        # Execution status
        self.executing_trajectory = False
        self.trajectory_points = []
        self.current_point_index = 0
        
        rospy.loginfo("ArmPi Trajectory Controller initialized")
    
    def trajectory_callback(self, msg):
        """
        Callback for receiving joint trajectory commands.
        """
        if self.executing_trajectory:
            rospy.logwarn("Already executing a trajectory, ignoring new command")
            return
        
        # Validate trajectory
        if not self._validate_trajectory(msg):
            rospy.logerr("Invalid trajectory received")
            return
        
        rospy.loginfo(f"Received trajectory with {len(msg.points)} points")
        
        # Store trajectory for execution
        self.trajectory_points = msg.points
        self.current_point_index = 0
        self.executing_trajectory = True
        
        # Start execution in a separate thread
        rospy.Timer(rospy.Duration(0.1), self._execute_trajectory, oneshot=True)
    
    def _validate_trajectory(self, trajectory_msg):
        """
        Validate the received trajectory message.
        """
        if not trajectory_msg.joint_names:
            rospy.logerr("No joint names specified in trajectory")
            return False
        
        if not trajectory_msg.points:
            rospy.logerr("No trajectory points specified")
            return False
        
        # Check if joint names match expected names
        if set(trajectory_msg.joint_names) != set(self.joint_names):
            rospy.logerr(f"Joint names mismatch. Expected: {self.joint_names}, Got: {trajectory_msg.joint_names}")
            return False
        
        return True
    
    def _execute_trajectory(self, event):
        """
        Execute the stored trajectory point by point.
        """
        if not self.executing_trajectory or self.current_point_index >= len(self.trajectory_points):
            self.executing_trajectory = False
            rospy.loginfo("Trajectory execution completed")
            return
        
        # Get current trajectory point
        point = self.trajectory_points[self.current_point_index]
        
        # Extract joint positions (convert from radians to degrees for ArmPi)
        joint_positions_rad = point.positions
        joint_positions_deg = [np.degrees(pos) for pos in joint_positions_rad]
        
        # Execute joint positions
        success = self._execute_joint_positions(joint_positions_deg)
        
        if success:
            # Update current joint positions
            self.current_joint_positions = joint_positions_rad
            self.current_point_index += 1
            
            # Schedule next point execution
            if self.current_point_index < len(self.trajectory_points):
                next_point = self.trajectory_points[self.current_point_index]
                if next_point.time_from_start.to_sec() > 0:
                    # Use specified timing
                    delay = next_point.time_from_start.to_sec() - point.time_from_start.to_sec()
                    rospy.Timer(rospy.Duration(delay), self._execute_trajectory, oneshot=True)
                else:
                    # Use default timing
                    rospy.Timer(rospy.Duration(0.1), self._execute_trajectory, oneshot=True)
            else:
                # Trajectory completed
                self.executing_trajectory = False
                rospy.loginfo("Trajectory execution completed")
        else:
            rospy.logerr(f"Failed to execute trajectory point {self.current_point_index}")
            self.executing_trajectory = False
    
    def _execute_joint_positions(self, joint_positions_deg):
        """
        Execute joint positions on the physical robot.
        """
        try:
            # Convert joint positions to ArmPi format
            # ArmPi expects: [base_angle, shoulder_angle, elbow_angle, wrist_angle]
            armpi_angles = joint_positions_deg[:4]  # First 4 joints
            
            # Execute using ArmPi SDK
            result = self.arm_ik.setPitchRangeMoving(armpi_angles, 100, 1)
            
            if result:
                rospy.logdebug(f"Executed joint positions: {armpi_angles}")
                return True
            else:
                rospy.logerr(f"Failed to execute joint positions: {armpi_angles}")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error executing joint positions: {e}")
            return False
    
    def _publish_joint_states(self):
        """
        Publish current joint states.
        """
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)  # Not implemented
        joint_state.effort = [0.0] * len(self.joint_names)    # Not implemented
        
        self.joint_state_pub.publish(joint_state)
    
    def run(self):
        """
        Main loop for the trajectory controller.
        """
        rospy.loginfo("ArmPi Trajectory Controller started")
        
        while not rospy.is_shutdown():
            # Publish current joint states
            self._publish_joint_states()
            
            # Sleep
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ArmPiTrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArmPi Trajectory Controller interrupted")
    except Exception as e:
        rospy.logerr(f"ArmPi Trajectory Controller error: {e}") 