#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
*******************Function:Car Drifting Movement Routine**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to stop running the program. If fail to stop, please try mupltiple time!
----------------------------------------------------------
''')

chassis = mecanum.MecanumChassis()

start = True
#Process before stopping 
def Stop(signum, frame):
    global start

    start = False
    print('closing...')
    chassis.set_velocity(0,0,0)  # close all motors 
    

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while start:
        chassis.set_velocity(50,180,0.3)
        time.sleep(3)
        chassis.set_velocity(50,0,-0.3)
        time.sleep(3)
    chassis.set_velocity(0,0,0)  # stop all motors
    print('turned off')

        
