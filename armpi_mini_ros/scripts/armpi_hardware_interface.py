#!/usr/bin/env python3
"""
ROS Hardware Interface for ArmPi Mini
Provides ROS control interface for the physical robot
"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
import tf2_ros
import tf2_geometry_msgs

# Import ArmPi SDK
import sys
sys.path.append('/home/pi/ArmPi_mini/')
from kinematics.arm_move_ik import *
from common.ros_robot_controller_sdk import Board

class ArmPiHardwareInterface:
    """
    ROS hardware interface for ArmPi Mini
    """
    
    def __init__(self):
        rospy.init_node('armpi_hardware_interface', anonymous=True)
        
        # Initialize hardware
        self.board = Board()
        self.AK = ArmIK()
        self.AK.board = self.board
        
        # Joint names (matching URDF)
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
        self.gripper_joint = 'gripper_joint'
        
        # Current joint states
        self.current_joint_positions = [0.0, 1.5708, 1.5708, 0.0]  # radians
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_efforts = [0.0, 0.0, 0.0, 0.0]
        
        # Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribers
        self.joint_command_sub = rospy.Subscriber('/joint_commands', JointTrajectory, self.joint_command_callback)
        self.pose_command_sub = rospy.Subscriber('/pose_commands', PoseStamped, self.pose_command_callback)
        
        # Action clients
        self.trajectory_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Control rate
        self.rate = rospy.Rate(50)  # 50Hz
        
        # Move to home position
        self.move_to_home()
        
        rospy.loginfo("ArmPi Hardware Interface initialized")

    def move_to_home(self):
        """Move robot to home position"""
        home_angles = [0.0, 1.5708, 1.5708, 0.0]  # radians
        self.execute_joint_positions(home_angles)
        rospy.sleep(2.0)

    def execute_joint_positions(self, joint_positions):
        """
        Execute joint positions on physical robot
        
        Args:
            joint_positions: List of joint angles in radians
        """
        try:
            # Convert radians to degrees and servo positions
            joint_degrees = [np.degrees(angle) for angle in joint_positions]
            
            # Map to servo commands (using ArmIK transform)
            θ1, θ2, θ3, θ4 = joint_degrees
            servos = self.AK.transformAngelAdaptArm(θ2, θ3, θ4, θ1)
            
            if servos:
                # Execute servo commands
                self.AK.servosMove((
                    servos["servo3"], 
                    servos["servo4"], 
                    servos["servo5"], 
                    servos["servo6"]
                ), 1000)  # 1 second movement time
                
                # Update current positions
                self.current_joint_positions = joint_positions
                
        except Exception as e:
            rospy.logerr(f"Failed to execute joint positions: {e}")

    def joint_command_callback(self, msg):
        """Handle joint trajectory commands"""
        if len(msg.points) > 0:
            point = msg.points[0]
            if len(point.positions) >= 4:
                joint_positions = list(point.positions[:4])
                self.execute_joint_positions(joint_positions)

    def pose_command_callback(self, msg):
        """Handle pose commands using inverse kinematics"""
        try:
            # Extract position and orientation
            position = msg.pose.position
            orientation = msg.pose.orientation
            
            # Convert to ArmPi coordinate system (cm)
            x = position.x * 100.0  # meters to cm
            y = position.y * 100.0
            z = position.z * 100.0
            
            # Use ArmIK to solve inverse kinematics
            result = self.AK.setPitchRangeMoving((x, y, z), 0, -90, 90, 1000)
            
            if result:
                servos, alpha, movetime, ik_result = result
                rospy.loginfo(f"Executed pose command: ({x}, {y}, {z})")
            else:
                rospy.logwarn(f"Failed to solve IK for pose: ({x}, {y}, {z})")
                
        except Exception as e:
            rospy.logerr(f"Failed to execute pose command: {e}")

    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names + [self.gripper_joint]
        joint_state.position = self.current_joint_positions + [0.0]  # gripper position
        joint_state.velocity = self.current_joint_velocities + [0.0]
        joint_state.effort = self.current_joint_efforts + [0.0]
        
        self.joint_state_pub.publish(joint_state)

    def publish_tf(self):
        """Publish robot transforms"""
        try:
            # Calculate forward kinematics for TF
            # This is a simplified version - you'd want to use proper FK
            base_to_end_effector = tf2_geometry_msgs.TransformStamped()
            base_to_end_effector.header.stamp = rospy.Time.now()
            base_to_end_effector.header.frame_id = "base_link"
            base_to_end_effector.child_frame_id = "end_effector"
            
            # Simplified transform (you'd calculate this properly)
            base_to_end_effector.transform.translation.x = 0.0
            base_to_end_effector.transform.translation.y = 0.0
            base_to_end_effector.transform.translation.z = 0.2
            
            base_to_end_effector.transform.rotation.x = 0.0
            base_to_end_effector.transform.rotation.y = 0.0
            base_to_end_effector.transform.rotation.z = 0.0
            base_to_end_effector.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(base_to_end_effector)
            
        except Exception as e:
            rospy.logerr(f"Failed to publish TF: {e}")

    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting ArmPi Hardware Interface")
        
        while not rospy.is_shutdown():
            try:
                # Publish joint states
                self.publish_joint_states()
                
                # Publish transforms
                self.publish_tf()
                
                # Sleep
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                self.rate.sleep()

def main():
    try:
        interface = ArmPiHardwareInterface()
        interface.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArmPi Hardware Interface stopped")

if __name__ == '__main__':
    main() 