import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
********Function: Hiwonder Raspberry Pi expansion board, control single PWM servo**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program, if it fails please try multiple times!
----------------------------------------------------------
''')
board = rrc.Board()
start = True

# close pre-processing
def Stop(signum, frame):
    global start
    start = False
    print('Closing...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while True:
        board.pwm_servo_set_position(2, [[2, 1500]]) # Set servo 1 pulse width to 1500
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 1000]]) # Set servo 1 pulse width to 1000
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 500]]) # Set servo 1 pulse width to 500
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 1000]]) # Set servo 1 pulse width to 1000
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 1500]]) # Set servo 1 pulse width to 1500
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 2000]]) # Set servo 1 pulse width to 2000
        time.sleep(1)
        board.pwm_servo_set_position(2, [[2, 2500]]) # Set servo 1 pulse width to 2500
        time.sleep(1)
        if not start:
            board.pwm_servo_set_position(2, [[2, 1500]]) # Set servo 1 pulse width to 1500
            time.sleep(1)
            print('Closed')
            break