#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import threading
import HiwonderSDK.Board as Board

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********Function: Hiwonder Raspberry Pi Expansion Board, PWM servo Control Routine**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to stop running the program. If fail to stop, please try mupltiple time!
----------------------------------------------------------
''')

start = True
#Process before stopping 
def Stop(signum, frame):
    global start

    start = False
    print('...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    
    while True:
        Board.setPWMServoPulse(1, 1500, 1000) # set the pulse width of No.1 servo as 1500 and the running time as 1000ms
        time.sleep(1)
        Board.setPWMServoPulse(1, 2500, 1000) # set the pulse width of No.1 servo as 2500 and the running time as 1000ms
        time.sleep(1)
        
        if not start:
            Board.setPWMServoPulse(1, 1500, 1000) # set the pulse width of No.1 servo as 1500 and the running time as 1000ms
            time.sleep(1)
            print('closed')
            break
    
    
        