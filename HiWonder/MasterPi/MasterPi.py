#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import queue
import Camera
import logging
import threading
import RPCServer
import MjpgServer
import numpy as np
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
import Functions.Running as Running
import Functions.Avoidance as Avoidance
import Functions.RemoteControl as RemoteControl

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

HWSONAR = Sonar.Sonar() #ultrasonic sensor 

QUEUE_RPC = queue.Queue(10)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)
    
voltage = 0.0

def voltageDetection():
    global voltage
    vi = 0
    dat = []
    previous_time = 0.00
    try:
        while True:
            if time.time() >= previous_time + 1.00 :
                previous_time = time.time()
                volt = Board.getBattery()/1000.0
                
                if 5.0 < volt  < 8.5:
                    dat.insert(vi, volt)
                    vi = vi + 1            
                if vi >= 3:
                    vi = 0
                    volt1 = dat[0]
                    volt2 = dat[1]
                    volt3 = dat[2]
                    voltage = (volt1+volt2+volt3)/3.0 
                    print('Voltage:','%0.2f' % voltage)
            else:
                time.sleep(0.01)
            
    except Exception as e:
        print('Error', e)
            
        
# un subthread
VD = threading.Thread(target=voltageDetection)
VD.setDaemon(True)
VD.start()


def startTruckPi():
    global HWEXT, HWSONIC
    global voltage
    
    previous_time = 0.00
    # After turning on the ultrosonic sensor, the light is turned off by defualt
    HWSONAR.setRGBMode(0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0,0,0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0,0,0))    
    HWSONAR.show()
    
    # The ultrasonic for game calling
    RemoteControl.HWSONAR = HWSONAR
    RemoteControl.init()
    RPCServer.HWSONAR = HWSONAR
    Avoidance.HWSONAR = HWSONAR
    
    RPCServer.QUEUE = QUEUE_RPC

    threading.Thread(target=RPCServer.startRPCServer,
                     daemon=True).start()  # rpc server 
    threading.Thread(target=MjpgServer.startMjpgServer,
                     daemon=True).start()  # streaming server

    
    loading_picture = cv2.imread('/home/pi/MiniPi/CameraCalibration/loading.jpg')
    cam = Camera.Camera()  # camera reading 
    cam.camera_open()
    Running.cam = cam

    while True:
        
        time.sleep(0.03)
        # execute the RPC command need to be executed in thread
        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                event, params, *_ = ret
                ret[2] = req(params)  # execute RPC command
                event.set()
            except:
                break
        #####
        # execute game program:
        try:
            if Running.RunningFunc > 0 and Running.RunningFunc <= 9:
                if cam.frame is not None:
                    frame = cam.frame.copy()
                    img = Running.CurrentEXE().run(frame)
                    if Running.RunningFunc == 9:
                        MjpgServer.img_show = np.vstack((img, frame))
                    else:
                        if voltage <= 7.2: 
                            MjpgServer.img_show = cv2.putText(img, "Voltage:%.1fV"%voltage, (420, 460), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
                        else:
                            MjpgServer.img_show = cv2.putText(img, "Voltage:%.1fV"%voltage, (420, 460), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
                else:
                    MjpgServer.img_show = loading_picture
            else:
                MjpgServer.img_show = cam.frame
                #cam.frame = None
        except KeyboardInterrupt:
            print('RunningFunc1', Running.RunningFunc)
            break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startTruckPi()
