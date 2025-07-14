#!/usr/bin/python3
# coding=utf8
"""
Tracking Components Package

This package contains modular components for the advanced tracking system:
- GestureDetector: Hand gesture recognition using MediaPipe Hands
- FaceDetector: Face detection and recognition using MediaPipe
- MotionDetector: Motion detection and spotlight control
- DepthEstimator: Depth estimation and visualization
- ZoomController: Camera zoom functionality
"""

from .gesture_detector import GestureDetector
from .face_detector import FaceDetector
from .motion_detector import MotionDetector
from .depth_estimator import DepthEstimator
from .zoom_controller import ZoomController

__all__ = [
    'GestureDetector',
    'FaceDetector', 
    'MotionDetector',
    'DepthEstimator',
    'ZoomController'
]

__version__ = '1.0.0'
__author__ = 'ArmPi Mini Team' 