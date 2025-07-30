#!/usr/bin/python3
# coding=utf8
"""
Advanced Color Tracking with Jacobian-based Control
Implements smooth tracking using Jacobian controller and advanced path planning
"""

import sys
import cv2
import time
import math
import threading
import numpy as np
sys.path.append('/home/pi/ArmPi_mini/')
import common.misc as Misc
import common.yaml_handle as yaml_handle
from kinematics.arm_move_ik import *
from kinematics.jacobian_controller import JacobianController
from common.ros_robot_controller_sdk import Board

# Vision parameters
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Load color thresholds
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# Target color setting
__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())

# Find largest contour
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max

class AdvancedColorTracker:
    """
    Advanced color tracking system using Jacobian-based control
    """
    
    def __init__(self, board_instance, arm_ik_instance):
        self.board = board_instance
        self.AK = arm_ik_instance
        
        # Initialize Jacobian controller
        self.jacobian_controller = JacobianController(arm_ik_instance)
        
        # Vision parameters
        self.image_size = (320, 240)
        self.camera_intrinsics = self.load_camera_parameters()
        
        # Tracking state
        self.is_tracking = False
        self.target_detected = False
        self.last_detection_time = 0
        self.detection_timeout = 1.0  # seconds
        
        # Target state
        self.target_position_3d = np.array([0.0, 0.0, 0.0])
        self.target_velocity_3d = np.array([0.0, 0.0, 0.0])
        self.detection_history = []
        self.max_history_length = 10
        
        # Camera-to-robot transformation
        self.camera_height = 25.0  # cm above base
        self.camera_offset = np.array([0.0, 0.0, self.camera_height])
        
        # Tracking parameters
        self.tracking_distance = 15.0  # cm from camera
        self.approach_height = 20.0   # cm above table
        self.safety_limits = {
            'x_range': (-15, 15),
            'y_range': (-15, 15), 
            'z_range': (5, 30)
        }
        
        # Predictive tracking
        self.prediction_time = 0.2  # seconds ahead
        self.kalman_filter = self.init_kalman_filter()
        
        # Control gains
        self.jacobian_controller.set_control_gains(
            position_gain=3.0,
            orientation_gain=1.5,
            damping_factor=0.05
        )
        
        load_config()
        print("Advanced Color Tracker initialized")

    def load_camera_parameters(self):
        """Load camera intrinsic parameters"""
        # Simplified camera model - replace with actual calibration
        fx, fy = 400.0, 400.0  # focal lengths in pixels
        cx, cy = 160.0, 120.0  # principal point
        return {
            'fx': fx, 'fy': fy,
            'cx': cx, 'cy': cy,
            'width': self.image_size[0],
            'height': self.image_size[1]
        }

    def init_kalman_filter(self):
        """Initialize Kalman filter for object state estimation"""
        from cv2 import KalmanFilter
        
        # State: [x, y, z, vx, vy, vz] - position and velocity
        kf = KalmanFilter(6, 3)
        
        # Transition matrix (constant velocity model)
        dt = 1.0 / 30.0  # assume 30 FPS
        kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Measurement matrix (observe position only)
        kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], dtype=np.float32)
        
        # Process noise
        kf.processNoiseCov = 0.1 * np.eye(6, dtype=np.float32)
        
        # Measurement noise
        kf.measurementNoiseCov = 1.0 * np.eye(3, dtype=np.float32)
        
        # Error covariance
        kf.errorCovPost = 1.0 * np.eye(6, dtype=np.float32)
        
        return kf

    def pixel_to_3d(self, pixel_x, pixel_y, object_radius_pixels):
        """
        Convert pixel coordinates to 3D world coordinates
        
        Args:
            pixel_x, pixel_y: Object center in pixels
            object_radius_pixels: Object radius in pixels
            
        Returns:
            position_3d: [x, y, z] in robot coordinate frame (cm)
        """
        # Estimate distance based on object size (simple approach)
        # Assumes known object size in real world
        real_object_radius = 3.0  # cm (approximate ball/object radius)
        estimated_distance = (real_object_radius * self.camera_intrinsics['fx']) / object_radius_pixels
        
        # Convert pixel to normalized camera coordinates
        x_norm = (pixel_x - self.camera_intrinsics['cx']) / self.camera_intrinsics['fx']
        y_norm = (pixel_y - self.camera_intrinsics['cy']) / self.camera_intrinsics['fy']
        
        # Project to 3D in camera frame
        x_cam = x_norm * estimated_distance
        y_cam = y_norm * estimated_distance
        z_cam = estimated_distance
        
        # Transform to robot base frame
        # Assuming camera is mounted above robot looking down
        x_robot = x_cam
        y_robot = -y_cam  # Flip Y axis
        z_robot = self.camera_height - z_cam
        
        return np.array([x_robot, y_robot, z_robot])

    def update_target_state(self, pixel_x, pixel_y, radius):
        """Update target state with new detection"""
        current_time = time.time()
        
        # Convert to 3D coordinates
        position_3d = self.pixel_to_3d(pixel_x, pixel_y, radius)
        
        # Update Kalman filter
        measurement = position_3d.astype(np.float32).reshape(3, 1)
        self.kalman_filter.correct(measurement)
        self.kalman_filter.predict()
        
        # Get filtered state
        state = self.kalman_filter.statePost.flatten()
        self.target_position_3d = state[:3]
        self.target_velocity_3d = state[3:]
        
        # Update detection history
        self.detection_history.append({
            'time': current_time,
            'position': self.target_position_3d.copy(),
            'velocity': self.target_velocity_3d.copy()
        })
        
        if len(self.detection_history) > self.max_history_length:
            self.detection_history.pop(0)
        
        self.target_detected = True
        self.last_detection_time = current_time

    def predict_target_position(self):
        """Predict target position using motion model"""
        if not self.target_detected:
            return None
        
        # Simple linear prediction
        predicted_position = (self.target_position_3d + 
                            self.target_velocity_3d * self.prediction_time)
        
        # Apply safety limits
        predicted_position[0] = np.clip(predicted_position[0], 
                                      self.safety_limits['x_range'][0],
                                      self.safety_limits['x_range'][1])
        predicted_position[1] = np.clip(predicted_position[1],
                                      self.safety_limits['y_range'][0], 
                                      self.safety_limits['y_range'][1])
        predicted_position[2] = np.clip(predicted_position[2],
                                      self.safety_limits['z_range'][0],
                                      self.safety_limits['z_range'][1])
        
        return predicted_position

    def get_target_pose(self):
        """Get target pose for Jacobian controller"""
        if not self.target_detected:
            return None
        
        current_time = time.time()
        if current_time - self.last_detection_time > self.detection_timeout:
            self.target_detected = False
            return None
        
        # Get predicted position
        predicted_pos = self.predict_target_position()
        if predicted_pos is None:
            return None
        
        # Maintain tracking distance
        predicted_pos[2] = max(predicted_pos[2], self.approach_height)
        
        # Target orientation (point down)
        target_orientation = np.array([0.0, -math.pi/2, 0.0])  # Point down
        
        # Combine position and orientation
        target_pose = np.concatenate([predicted_pos, target_orientation])
        
        return target_pose

    def start_tracking(self):
        """Start Jacobian-based tracking"""
        if self.is_tracking:
            return
        
        self.is_tracking = True
        
        # Start Jacobian controller tracking
        self.jacobian_controller.start_tracking(self.get_target_pose)
        
        print("Advanced tracking started")

    def stop_tracking(self):
        """Stop tracking"""
        if not self.is_tracking:
            return
        
        self.is_tracking = False
        self.jacobian_controller.stop_tracking()
        
        print("Advanced tracking stopped")

    def process_image(self, img):
        """
        Process image and update tracking
        
        Args:
            img: Input image from camera
            
        Returns:
            processed_img: Image with tracking visualization
        """
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        
        # Resize and blur
        frame_resize = cv2.resize(img_copy, self.image_size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        
        # Convert to LAB color space
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        # Find target color
        area_max = 0
        areaMaxContour = None
        detect_color = 'None'
        
        for color in lab_data:
            if color in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                       (lab_data[color]['min'][0],
                                        lab_data[color]['min'][1],
                                        lab_data[color]['min'][2]),
                                       (lab_data[color]['max'][0],
                                        lab_data[color]['max'][1],
                                        lab_data[color]['max'][2]))
                
                # Morphological operations
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                
                # Find contours
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour_temp, area_max_temp = getAreaMaxContour(contours)
                
                if area_max_temp > area_max:
                    area_max = area_max_temp
                    areaMaxContour = areaMaxContour_temp
                    detect_color = color
        
        # Process detection
        if area_max > 500 and areaMaxContour is not None:
            # Get object properties
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
            
            # Scale back to original image coordinates
            center_x_orig = int(Misc.map(center_x, 0, self.image_size[0], 0, img_w))
            center_y_orig = int(Misc.map(center_y, 0, self.image_size[1], 0, img_h))
            radius_orig = int(Misc.map(radius, 0, self.image_size[0], 0, img_w))
            
            # Update target state
            self.update_target_state(center_x, center_y, radius)
            
            # Visualization
            cv2.circle(img, (center_x_orig, center_y_orig), radius_orig, 
                      range_rgb[detect_color], 2)
            cv2.circle(img, (center_x_orig, center_y_orig), 3, (0, 255, 0), -1)
            
            # Show predicted position
            if self.is_tracking:
                predicted_pos = self.predict_target_position()
                if predicted_pos is not None:
                    # Project 3D prediction back to image for visualization
                    pred_pixel_x = predicted_pos[0] * self.camera_intrinsics['fx'] / self.tracking_distance + self.camera_intrinsics['cx']
                    pred_pixel_y = -predicted_pos[1] * self.camera_intrinsics['fy'] / self.tracking_distance + self.camera_intrinsics['cy']
                    
                    pred_x_orig = int(Misc.map(pred_pixel_x, 0, self.image_size[0], 0, img_w))
                    pred_y_orig = int(Misc.map(pred_pixel_y, 0, self.image_size[1], 0, img_h))
                    
                    if 0 <= pred_x_orig < img_w and 0 <= pred_y_orig < img_h:
                        cv2.circle(img, (pred_x_orig, pred_y_orig), 5, (255, 255, 0), -1)
                        cv2.line(img, (center_x_orig, center_y_orig), 
                                (pred_x_orig, pred_y_orig), (255, 255, 0), 2)
            
            # Display tracking info
            info_text = f"Target: {detect_color} | 3D: ({self.target_position_3d[0]:.1f}, {self.target_position_3d[1]:.1f}, {self.target_position_3d[2]:.1f})"
            cv2.putText(img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if self.is_tracking:
                current_pose = self.jacobian_controller.get_current_pose()
                pose_text = f"EE: ({current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f})"
                cv2.putText(img, pose_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        else:
            # No detection
            if time.time() - self.last_detection_time > self.detection_timeout:
                self.target_detected = False
        
        return img

    def execute_smooth_approach(self, target_position, approach_type='quintic'):
        """
        Execute smooth approach to target using path planning
        
        Args:
            target_position: [x, y, z] target position
            approach_type: Type of trajectory ('quintic', 'spline', 'circular')
        """
        if self.is_tracking:
            self.stop_tracking()
        
        # Get current pose
        current_pose = self.jacobian_controller.get_current_pose()
        
        # Plan approach trajectory
        if approach_type == 'circular':
            # Circular approach from above
            center = [target_position[0], target_position[1]]
            radius = 5.0  # cm
            height = target_position[2] + 10.0
            duration = 3.0
            
            trajectory = self.jacobian_controller.circular_trajectory(
                center, radius, height, duration
            )
            
            # Execute circular trajectory
            self.jacobian_controller.execute_trajectory(trajectory, duration)
            
        else:
            # Point-to-point approach
            # Add intermediate waypoint above target
            intermediate_pose = current_pose.copy()
            intermediate_pose[:3] = [target_position[0], target_position[1], target_position[2] + 10.0]
            
            # Create multi-segment trajectory
            segment1 = self.jacobian_controller.smooth_point_to_point(
                current_pose, intermediate_pose, 2.0, approach_type
            )
            
            final_pose = intermediate_pose.copy()
            final_pose[:3] = target_position
            
            segment2 = self.jacobian_controller.smooth_point_to_point(
                intermediate_pose, final_pose, 1.5, approach_type
            )
            
            # Execute trajectory segments
            self.jacobian_controller.execute_trajectory(segment1, 2.0)
            time.sleep(0.1)
            self.jacobian_controller.execute_trajectory(segment2, 1.5)

    def emergency_stop(self):
        """Emergency stop - halt all motion"""
        self.stop_tracking()
        print("Emergency stop activated")

# Global instances
board = None
AK = None
tracker = None

def init():
    """Initialize the advanced tracking system"""
    global board, AK, tracker
    
    print("Initializing Advanced Color Tracking")
    
    # Initialize hardware
    board = Board()
    AK = ArmIK()
    AK.board = board
    
    # Initialize tracker
    tracker = AdvancedColorTracker(board, AK)
    
    # Move to initial position
    AK.setPitchRangeMoving((0, 10, 20), -90, -90, 90, 1500)
    time.sleep(2)

def start():
    """Start advanced tracking"""
    global tracker
    if tracker:
        tracker.start_tracking()

def stop():
    """Stop advanced tracking"""
    global tracker
    if tracker:
        tracker.stop_tracking()

def exit():
    """Exit advanced tracking"""
    global tracker
    if tracker:
        tracker.emergency_stop()

def run(img):
    """Process frame for advanced tracking"""
    global tracker
    if tracker:
        return tracker.process_image(img)
    return img

if __name__ == '__main__':
    # Test the advanced tracking system
    init()
    start()
    
    # Simulate camera feed
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    
    try:
        while True:
            ret, img = cap.read()
            if ret:
                frame = run(img)
                frame_resize = cv2.resize(frame, (640, 480))
                cv2.imshow('Advanced Color Tracking', frame_resize)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                elif key == ord('s'):  # Start tracking
                    start()
                elif key == ord('q'):  # Stop tracking
                    stop()
                elif key == ord('a'):  # Approach target
                    if tracker and tracker.target_detected:
                        tracker.execute_smooth_approach(tracker.target_position_3d, 'quintic')
            
            time.sleep(0.03)
            
    except KeyboardInterrupt:
        pass
    finally:
        exit()
        cv2.destroyAllWindows()
        cap.release()