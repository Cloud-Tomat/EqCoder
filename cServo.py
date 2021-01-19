#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 21:11:46 2021

@author: nicolas
"""

import time
import threading
import numpy as np

class cServo:
    
    def __init__(self,serialPort):
        if (serialPort is None):
            print("No Serial Port : Simulation")
            self.simu=True
        else:
            self.ser=serialPort
            time.sleep(3)
            self.sendCommand(1)

            self.simu=True
        
        #PID params 
        self.Kp=8
        self.Kd=5000
        
        #Target Speed
        self.speedTh=0.6444027869097377/1000
        
        #Speed from speedTh/speedfactor to speedTh*SpeedFactor
        self.speedFactor=2
        
        #depth of analysis
        self.limit=5

        #Servo loop time in ms
        self.timeStep=1000
        
        #Terminate servo thread
        self.terminate=False
        
        #result of move calc between frames
        self.results=None
        
        self.lastTime=0


    def sendCommand(self,command):
        if (not self.simu):
            cmdTxt=bytes("%5.2f#"%(command), 'utf-8')
            self.ser.write(cmdTxt)

    def startServoThread(self):
        servoThread = threading.Thread(target=self.servo)
        servoThread.start()

    def servo(self):
        while not self.terminate:
            if (not self.results is None):
                while (self.results[-1,0]-self.lastTime)<self.timeStep and not self.terminate:
                    time.sleep(0.005)
                self.update()
            else:
                    time.sleep(0.005)                
        self.sendCommand(1)

    def addResult(self,result):
        if (self.results is None):
            self.results=np.array([result])
        else:
            self.results=np.vstack((self.results,result))

    def update(self):
        print(self.results)
        self.lastTime=self.results[-1,0]
        results=np.copy(self.results)
        self.results=None
        
        print(len(results))
        
        x=results[:,0]
        y=results[:,1]
        v=results[:,3]
           
        dt=np.mean(x)
        lastPos=np.mean(y)
        speed=np.mean(v)
            
        #Position error
        erreurP=lastPos-dt*self.speedTh
        
        #Derivative of pos error estimate
        erreurH=[y[i]-x[i]*self.speedTh for i in range(len(x))]
        z = np.polyfit(x,erreurH, 1)
        p = np.poly1d(z)
        erreurD=p(1)-p(0)

        erreur=self.Kp*erreurP +self.Kd*erreurD

        if (erreur<-self.limit): erreur=-self.limit
        if (erreur>self.limit): erreur=self.limit
            

        command=(1+abs(erreur)/self.limit)*self.speedFactor/2
        if (erreur>0) : command=1/command
        self.sendCommand(command)

                    
                
