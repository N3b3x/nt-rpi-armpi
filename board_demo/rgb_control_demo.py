import time
import signal
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
********Function: Hiwonder Raspberry Pi Expansion Board, RGB Light Control**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program, if it fails please try multiple times!
----------------------------------------------------------
''')

start = True
# Pre-shutdown processing
def Stop(signum, frame):
    global start

    start = False
    print('Closing...')
board = rrc.Board()
# First turn off all lights
board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
signal.signal(signal.SIGINT, Stop)

while True:
    # Set 2 lights to red
    board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    time.sleep(1)
    
    # Set 2 lights to green
    board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    time.sleep(1)
    
    # Set 2 lights to blue
    board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    time.sleep(1)
    
    # Set 2 lights to yellow
    board.set_rgb([[1, 255, 255, 0], [2, 255, 255, 0]])
    time.sleep(1)

    if not start:
        # Turn off all lights
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        print('Closed')
        break
