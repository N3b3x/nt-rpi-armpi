#!/usr/bin/env python3
# encoding:utf-8

import sys
import time
import numpy as np
from math import sqrt, radians, cos, sin, atan2, degrees, acos, pow
import logging
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics.inversekinematics import *
import common.yaml_handle as yaml_handle

# Setup logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Obtain the servo deviation of the robotic arm
deviation_data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)

# The robotic arm rotates based on the angle calculated by inverse kinematics
ik = IK()

class ArmIK:
    servo3Range = (500, 2500.0, 0, 180.0)  # pulse width, angle
    servo4Range = (500, 2500.0, 0, 180.0)
    servo5Range = (500, 2500.0, 0, 180.0)
    servo6Range = (500, 2500.0, 0, 180.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        logger.debug(f"Transform angles: theta3={theta3}, theta4={theta4}, theta5={theta5}, theta6={theta6}")

        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        servo3 = max(self.servo3Range[0], min(self.servo3Range[1], servo3))
        logger.debug(f"servo3 pulse: {servo3}")

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        servo4 = max(self.servo4Range[0], min(self.servo4Range[1], servo4))
        logger.debug(f"servo4 pulse: {servo4}")

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + (90.0 - theta5) * self.servo5Param))
        servo5 = max(self.servo5Range[0], min(self.servo5Range[1], servo5))
        logger.debug(f"servo5 pulse: {servo5}")

        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param)) + self.servo6Range[0]
        servo6 = max(self.servo6Range[0], min(self.servo6Range[1], servo6))
        logger.debug(f"servo6 pulse: {servo6}")

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime=None):
        logger.debug(f"Moving servos: {servos} with movetime={movetime}")
        time.sleep(0.02)
        if movetime is None:
            max_d = max(abs(deviation_data[str(i+3)]) for i in range(4))
            movetime = int(max_d)
        # Explicitly convert to int
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[3, int(servos[0]+deviation_data['3'])]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[4, int(servos[1]+deviation_data['4'])]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[5, int(servos[2]+deviation_data['5'])]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[6, int(servos[3]+deviation_data['6'])]])
        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da=1):
        x, y, z = coordinate_data
        logger.debug(f"setPitchRange: searching pitch {alpha1} to {alpha2} at (x={x}, y={y}, z={z})")
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):
            logger.debug(f"Trying pitch angle: {alpha}")
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                logger.debug(f"Found IK: {result}")
                servos = self.transformAngelAdaptArm(result['theta3'], result['theta4'], result['theta5'], result['theta6'])
                if servos:
                    logger.debug(f"Valid servo config for pitch {alpha}: {servos}")
                    return servos, alpha, result
        logger.warning(f"No IK solution for (x={x}, y={y}, z={z}) in pitch range")
        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        x, y, z = coordinate_data
        logger.debug(f"setPitchRangeMoving: coordinate_data={coordinate_data}, alpha={alpha}, range=({alpha1}, {alpha2})")
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        
        # Check if we have valid results
        if result1 and result1 is not False:
            data = result1
            if result2 and result2 is not False and abs(result2[1] - alpha) < abs(result1[1] - alpha):
                data = result2
        elif result2 and result2 is not False:
            data = result2
        else:
            logger.warning(f"No valid pitch found for (x={x}, y={y}, z={z})")
            return False, False, False, False
            
        servos, alpha, result = data[0], data[1], data[2]
        logger.debug(f"Final servo commands: {servos}")
        movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
        return servos, alpha, movetime, result

if __name__ == "__main__":

    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    print(AK.setPitchRangeMoving((12, 0, 0.5), 0, -90, 90, 1500))
