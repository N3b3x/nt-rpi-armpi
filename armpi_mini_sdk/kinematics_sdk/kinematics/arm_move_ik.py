#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics.inversekinematics import *
import common.yaml_handle as yaml_handle

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
        # Adapt to different servos
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # Convert the angles calculated by inverse kinematics into pulse width values corresponding to the servos
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0]:
            logger.info('servo3(%s) out of range (%s, %s)', servo3, self.servo3Range[0], self.servo3Range[1])
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s) out of range (%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + (90.0 - theta5) * self.servo5Param)) 
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            logger.info('servo5(%s) out of range (%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False

        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param)) + self.servo6Range[0]
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s) out of range (%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False
        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime=None):
        # Drive servos 3, 4, 5, and 6
        time.sleep(0.02)
        if movetime is None:
            max_d = 0
            for i in range(0, 4):
                d = abs(deviation_data['{}'.format(i+3)])
                if d > max_d:
                    max_d = d
            movetime = int(max_d*1)
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[3, servos[0]+deviation_data['3']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[4, servos[1]+deviation_data['4']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[5, servos[2]+deviation_data['5']]])
        self.board.pwm_servo_set_position(float(movetime)/1000.0, [[6, servos[3]+deviation_data['6']]])

        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da=1):
        # Specify the coordinate 'coordinate_data' and the range of pitch angles 'alpha1' and 'alpha2', automatically find a suitable solution within the range
        # If there is no solution, return 'False', otherwise return the corresponding servo angle and pitch angle
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos:
                    return servos, alpha

        return False

    def setPitchRanges(self, coordinate_data, alpha, alpha1, alpha2, d=0.01):
        # Specify the coordinate to 'coordinate_data', and the pitch angle to 'alpha' ranging from 'alpha1' and 'alpha2', then it automatically finds a suitable solution within the range
        x, y, z = coordinate_data
        a_range = abs(int(abs(alpha1 - alpha2)/d)) + 1
        for i in range(a_range):
            if i % 2:
                alpha_ = alpha + (i + 1)/2*d
            else:
                alpha_ = alpha - i/2*d
                if alpha_ < alpha1:
                    alpha_ = alpha2 - i/2*d
            result = ik.getRotationAngle((x, y, z), alpha_)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                return servos, alpha_

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        # Specify the coordinate to 'coordinate_data', and the pitch angle to 'alpha' ranging from 'alpha1' and 'alpha2', then it automatically finds a suitable solution within the range to rotate to the target position
        # If there is no solution, return 'False', otherwise return the servo angle, pitch angle, and runtime
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1:
            data = result1
            if result2:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]
        movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
        return servos, alpha, movetime

if __name__ == "__main__":
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    print(AK.setPitchRangeMoving((12, 0, 0.5), 0, -90, 90, 1500))
