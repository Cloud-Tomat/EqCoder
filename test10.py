#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 19 13:12:05 2020

@author: nicolas
"""

SIDERAL_SPEED=15.041

import numpy as np 
import cv2 
import math as math

import exifread
from datetime import datetime
import time
import matplotlib.pyplot as plt
import select
import sys

import serial
import threading

import cFrameBuffer
import cServo

terminate=False
path="/mnt/data/sandbox/eq/s1/"

def heardEnter():
    i,o,e = select.select([sys.stdin],[],[],0.0001)
    for s in i:
        if s == sys.stdin:
            input = sys.stdin.readline()
            return True
    return False




def capFrame(cap):
    execTime=0
    i=0
    while(execTime<0.03):
        i+=1
        e1 = cv2.getTickCount()
        cap.grab()
        e2 = cv2.getTickCount()
        execTime= ((e2 - e1)/ cv2.getTickFrequency())
    #print("%d\t%4.4f"%(i,execTime))
    
    msecCounter = cap.get(cv2.CAP_PROP_POS_MSEC)
    ret,frame = cap.retrieve()    
    return msecCounter,frame


def graph():
    plt.ion()
    speedTh=0.6444027869097377/1000 #pix/ms
    scaleFactor=15.04/(speedTh*1000) #Arcsec/pix

    print("ScaleFactor ",scaleFactor)
    graphDepth=500
    while (not terminate):
        if (not results is None):
            toGraph=np.array(results)
            x=toGraph[:,0]/1000.0
            y=toGraph[:,1]
            y=y[-graphDepth:]
            x=x[-graphDepth:]
            y=[(y[i]-speedTh*x[i]*1000)*scaleFactor for i in range(len(y))]
            
            plt.plot(x,y)
            plt.draw()
            plt.pause(1)
            plt.clf()

def interactive():
    global Kp,Kd,terminate
    while not terminate:
        command = input(">")
        exec(command, globals())
        print("Kp=%2.3f\tKd=%2.3f"%(Kp,Kd))


def ProcessDirectory(path,outputPath):
    global results
    global terminate
    results=None
    terminate=False

    #----------
    #Init Serial
    if True:
        ser = serial.Serial('/dev/ttyACM0',115200)
    #-----------

    
    #-----------
    #Init video

    cap = cv2.VideoCapture(1)
    #Minimul buffer size to get the most recent frame from catpure
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    print(cap.get(cv2.CAP_PROP_BUFFERSIZE))

    #Max resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #-----------
    

    
    
#-----------
#Start thread
    #interactiveThread = threading.Thread(target=interactive)
    #interactiveThread.start()

    graphThread = threading.Thread(target=graph)
    graphThread.start()
    
    servo=cServo.cServo(None)
    servo.startServoThread()
#----------

    time1, frame1 = capFrame(cap)
    frameBuffer=cFrameBuffer.cFrameBuffer(time,frame1)

    while(not heardEnter()):

        time2,frame2=capFrame(cap)

        dist,angle,speed=frameBuffer.update(time2,frame2)
        result=[time2-time1,dist,angle,speed]
        servo.addResult(result)

        if results is None:
           results=[result]
        else:
           results.append(result)


    terminate=True
    results=np.array(results)

    outputRoot=str(datetime.now()).replace(":","-")+"_Eq_calib_"
    outputFile=outputPath+outputRoot+"raw_results.csv"
  
    
    print ("saving to "+outputFile)

    np.savetxt(outputFile,  
               results, 
               delimiter =",",  
               fmt ='% s',
               header="time,dist,angle,speed",
               comments="") 
    return outputFile,outputRoot

if __name__ == '__main__':
    ProcessDirectory("","/mnt/data/sandbox/eqEncoder/video/")
