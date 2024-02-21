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
********Function: Hiwonder Raspberry Pi Expansion Board, Motor Control Routine**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to stop running the program. If fail to stop, please try mupltiple time!
----------------------------------------------------------
''')

# stop all motors 
def MotorStop():
    Board.setMotor(1, 0) 
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)

start = True
#Process before stopping 
def Stop(signum, frame):
    global start

    start = False
    print('closing...')
    MotorStop()  # stop all motors
    

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    
    while True:
        Board.setMotor(1, 35)  #Set No.1 motor at the speed of 35
        time.sleep(1)
        Board.setMotor(1, 60)  #60 Set No.1 motor at the speed of 60
        time.sleep(2)
        Board.setMotor(1, 90)  #90 Set No.1 motor at the speed of 90
        time.sleep(3)     
        
        if not start:
            MotorStop()  # stop all motors
            print('closed')
            break
    
    
        