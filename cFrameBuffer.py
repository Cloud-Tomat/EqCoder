#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 20:55:04 2021

@author: nicolas
"""

import numpy as np
import cv2 
import math

class cFrameBuffer:
    #Number of frame to compare (key frames)
    bufferSize=3
    #distance between key frames
    distLimit=10

    
    def __init__(self,timeStamp,frame):
        self.frame=[None]*self.bufferSize
        self.frame[0]=  frame
        
        self.timeStamp=[None]*self.bufferSize
        self.timeStamp[0]=timeStamp
        
        self.dist=[None]*self.bufferSize
        self.dist[0]=0
        
        self.angle=[None]*self.bufferSize
        self.angle[0]=0
        
        self.initialized=False
        self.framePointer=1
        self.initPointer=1
        
        self.lcTimeStamp=[None]*self.bufferSize
        self.lcDist=[None]*self.bufferSize
        
        #distance of last pixel increment
        self.curDistList=[None]*self.bufferSize
        self.speed=[None]*self.bufferSize
        
        self.lastTimeStamp=0
        
        self.lastDistMean=0.0
        self.timeOfLastChange=0.0
    
    
    def CalcMove(self,query,train):
        query_img = query
        train_img = train
        
        factor=2
        offset=60
        
         #for debug
        if factor!=1:
            width = int(query_img.shape[1] * factor)
            height = int(query_img.shape[0] * factor)
                # dsize
            dsize = (width, height)     
            query_img=cv2.resize(query_img,dsize)
            train_img=cv2.resize(train_img, dsize)
        
        # Convert it to grayscale conda install --name eq
        query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY) 
        train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY) 
        
        #crop query img center
        w = query_img.shape[1] 
        h = query_img.shape[0]
    
        startQuery=(int(w/2-w*0.1),int(h/2-h*0.1))
        endQuery=(int(w/2+w*0.1),int(h/2+h*0.1)) 
        
        startTrain=(int(w/2-w*0.1-offset),int(h/2-h*0.1-offset))
        endTrain=(int(w/2+w*0.1)+offset,int(h/2+h*0.1)+offset) 
        #startTrain=(int(w/2-w*0.5),int(h/2-h*0.5))
        #endTrain=(int(w/2+w*0.5),int(h/2+h*0.5)) 
        
        roiQuery=query_img_bw[startQuery[1]:endQuery[1],startQuery[0]:endQuery[0]]
        train_img_bw=train_img_bw[startTrain[1]:endTrain[1],startTrain[0]:endTrain[0]]
    
    
    
        #    query_img=cv2.rectangle(query_img,start, end, 255, 2)
        img = train_img_bw
        img2 = img.copy()
        template = roiQuery
        w, h = template.shape[::-1]
        # All the 6 methods for comparison in a list
        methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        
        img = img2.copy()
        method = eval(methods[1])
        # Apply template Matching
        res = cv2.matchTemplate(img,template,method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        
        dstX=float(top_left[0]-offset)/factor
        dstY=float(top_left[1]-offset)/factor
        
        dst=math.sqrt(dstX**2+dstY**2)
        angle=math.atan2(dstY,dstX)*360/math.pi
        #print (str(dstX)+"\t"+str(dstY)+"\t"+str(dst)+"\t"+str(angle))
        
        bottom_right = (top_left[0] + w, top_left[1] + h)
        
        if (angle>0) : dst=-dst
        
        return (dst,angle)
    
    def update(self,timeStamp,frame):
    
        angleList=[]
        dt=timeStamp-self.lastTimeStamp
        self.lastTimeStamp=timeStamp
        
        
        for i in range(self.initPointer):
            dist,angle=self.CalcMove(self.frame[i],frame)
            absDist=dist+self.dist[i]

            skipInit=False
            speedUpdate=False #For debug            
            if (not self.curDistList[i] is None):
                #Capture instant of pixel move
                if (absDist!=self.curDistList[i]):
                     #Check init stuff
                    if not self.curDistList[i] is None and not self.lcDist[i] is None:
                        #Add a time stamp condition 'blind zone" to avoid rebounce 
                        if (timeStamp-self.lcTimeStamp[i])>1000.0:
                            #Speed Estimate
                            self.speed[i]=(absDist-self.lcDist[i])/(timeStamp-self.lcTimeStamp[i])
                            speedUpdate=True
                        else:
                            skipInit=True
                    if not skipInit:
                        self.lcDist[i]=absDist
                        self.lcTimeStamp[i]=timeStamp

            self.curDistList[i]=absDist
            angleList.append(angle)

            #Distance from last in 
            if i==((self.framePointer-1+self.bufferSize)%self.bufferSize):
                distPrev=dist
                #print("%d\t%3.2f"%(i,distPrev))
        
        speed=[i for i in self.speed if not i is None]
        if speed!=[] :
            meanSpeed=np.mean(speed)
        else:
            meanSpeed=0.0



        #if speedUpdate:
        #    print(timeStamp,self.speed,meanSpeed)

            
        
        #Evaluate Mean Position
        distList=[i for i in self.curDistList if not i is None]
        distMean=np.mean(distList)
        angleMean=np.mean(angle)
        
      
        
        #print("%5.2f"%(dt),self.dist,distList,distMean)
        #print("%5.2f\t%5.2f\t%5.2f"%(dt,distMean,meanSpeed))
        #print(meanSpeed)
        #(self.initPointer<self.bufferSize) or 

        if (abs(distPrev)>self.distLimit): # and False:
            self.frame[self.framePointer]=frame
            self.timeStamp[self.framePointer]=timeStamp
            self.dist[self.framePointer]=distMean
            self.angle[self.framePointer]=angleMean
            self.framePointer=(self.framePointer+1)%self.bufferSize
            self.initPointer=min(self.initPointer+1,self.bufferSize)
            self.curDistList[self.framePointer]=None
            #print("Frame added")
            
        return (distMean,angleMean,meanSpeed)            
        