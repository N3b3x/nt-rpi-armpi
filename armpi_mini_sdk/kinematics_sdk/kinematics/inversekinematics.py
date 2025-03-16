#!/usr/bin/env python3
# encoding: utf-8
# 4-DOF robotic arm inverse kinematics: Given the corresponding coordinates (X, Y, Z) and pitch angle, calculate the rotation angle of each joint
# 2022/09/26 
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Servos are numbered from bottom to top
    # common parameters refer to the linkage parameters of a 4-DOF robotic arm
    l1 = 8.80    # the distance(cm) from the center of the robotic arm's base to the center axis of the second servo
    l2 = 5.80    # the distance(cm) from the second servo to the third servo
    l3 = 6.20    # the distance(cm) from the third servo to the forth servo
    l4 = 9.50    # the distance(cm) from the forth servo to the edd effector of the robotic arm, which refers to the position of the gripper when it is fully closed
    
    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # Change the linkage lengths of a robotic arm to adapt the different lengths of the robotic arm with the same structure
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):
        # obtain current set linkage length
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # Given a specified coordinate and pitch angle, return the angle that each joint should rotate, or False if there is no solution
        # 'coordinate_data' is the end effector coordinate of the gripper in the unit of cm, transmitted in tuple form, for example (0, 5, 10)
        # 'Alpha' is the angle between the gripper and the horizontal plane, in degrees
        # Set the end effector of the gripper to P(X, Y, Z), the origin which is the projection of the pan-tilt's center on the ground to O, and the  projection of P on the ground to P_
        # The intersection of l1 and l2 is A, the intersection of l2 and l3 is B, and the intersection of l3 and l4 is C
        # CD is perpendicular to PD, CD is perpendicular to the z-axis, and the pitch angle 'Alpha' is the angle between DC and PC. AE is perpendicular to DP_, and E is on DP_; CF is perpendicular to AE, and F is on AE
        # Angle representation: for example, the angle between AB and BC is represented as ABC
        X, Y, Z = coordinate_data
        # calculate the rotation angle of the base
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) # calculate the distance form 'P_' to the origin '0'
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) # When the pitch angle is positive, PD is greater than 0; when the pitch angle is negative, PD is less than 0
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # the sum of any two sides of a triangle must be greater than the third side 
            logger.debug('l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False
        # calculate theta4
        cos_ABC = round((pow(self.l2, 2) + pow(self.l3, 2) - pow(AC, 2))/(2*self.l2*self.l3), 4) # Law of Cosines 
        if abs(cos_ABC) > 1:
            logger.debug('abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) # calculate the angle in radians using inverse trigonometric functions 
        theta4 = 180.0 - degrees(ABC)
        # calculate theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # Law of Cosines
        if abs(cos_BAC) > 1:
            logger.debug('abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))
        # theta3
        theta3 = Alpha - theta5 + theta4
       
        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # return to the angle dictionary when there is a solution
            
if __name__ == '__main__':
    ik = IK()
    ik.setLinkLength(L1=ik.l1 + 1.3)
    print('Link Length：', ik.getLinkLength())
    print(ik.getRotationAngle((0, ik.l4, ik.l1 + ik.l2 + ik.l3), 0))
