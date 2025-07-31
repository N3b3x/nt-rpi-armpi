#!/usr/bin/python3
# coding=utf8
import time
from common.ros_robot_controller_sdk import Board
from common.action_group_control import ActionGroupController

board = Board()
AGC = ActionGroupController(board)

print('''
**********************************************************
************Function:Hiwonder Raspberry Pi Expansion Board, Action Group Control************
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
Tips:
 * Press 'Ctrl+C' to exit this program, if it fails, please try a few more times!
----------------------------------------------------------
''')

# the action groups need to be saved in the path '/home/pi/ArmPi_mini/action_groups'
AGC.runAction('1')  # The parameter should be the name of the action group in string format, without the file extension.
AGC.runAction('2')