# -*- coding: utf-8 -*-
"""
Created on Sat Oct 22 15:59:54 2022

@author: Ali
"""

import time
import serial
import numpy as np
from vpython import *
import os
import random
import math

interval=10 #print Q every 10 seconds

arduino_data = serial.Serial('com6', 115200)
time.sleep(1)

def printit():
  threading.Timer(interval, printit).start()
  print ("Q(r,p,y) = ")
  print(Q_rpy(roll , pitch , yaw))
  print("***********")
  print("Q(w,x,y,z) = ")
  print(Q_quaternion(w,x,y,z))
  print("**********************************************************")

toRad = np.pi/180.0
toDeg = 1/toRad


# Simulate pong game using VPython here

roomX=12
roomY=10
roomZ=16
wallT=.5
wallColor=vector(1,1,1)
wallOpacity=.8
frontOpacity=.1
marbleR=.5
ballColor=vector(0,0,1)
 
myFloor=box(size=vector(roomX,wallT,roomZ),pos=vector(0,-roomY/2,0),color=wallColor,opacity=wallOpacity)
myCeiling=box(size=vector(roomX,wallT,roomZ),pos=vector(0,roomY/2,0),color=wallColor,opacity=wallOpacity)
leftWall=box(size=vector(wallT,roomY,roomZ),pos=vector(-roomX/2,0,0),color=wallColor,opacity=wallOpacity)
rightWall=box(size=vector(wallT,roomY,roomZ),pos=vector(roomX/2,0,0),color=wallColor,opacity=wallOpacity)
backWall=box(size=vector(roomX,roomY,wallT),pos=vector(0,0,-roomZ/2),color=wallColor,opacity=wallOpacity)
frontWall=box(size=vector(roomX,roomY,wallT),pos=vector(0,0,roomZ/2),color=wallColor,opacity=frontOpacity)
marble=sphere(color=ballColor,radius=marbleR)
 
paddleX=2
paddleY=2
paddleZ=.2
paddleOpacity=.8
paddleColor=vector(0,.8,.6)
paddleHit=vector(1,.8,.6)
paddle=box(size=vector(paddleX,paddleY,paddleZ),pos=vector(0,0,roomZ/2),color=paddleColor,opacity=paddleOpacity)

marbleX=0
deltaX=.03
 
marbleY=0
deltaY=.03
 
marbleZ=0
deltaZ=.03

scale=120.0
 
# Def your function here
def Q_rpy(r , p , y): ## r,p,y should be in radian Q_xyz
    qx=np.matrix([(1,0,0),(0,np.cos(r),-np.sin(r)),(0,np.sin(r),np.cos(r))] , dtype=np.float32)
    qy=np.matrix([(np.cos(p),0,np.sin(p)),(0,1,0),(-np.sin(p),0,np.cos(p))] , dtype=np.float32)
    qz=np.matrix([(np.cos(y),-np.sin(y),0),(np.sin(y),np.cos(y),0),(0,0,1)] , dtype=np.float32)
    q=qx*qy*qz
    return q

def Q_quaternion(q0,q1,q2,q3):
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    q = np.array([[r00, r01, r02],
                 [r10, r11, r12],
                 [r20, r21, r22]],dtype=np.float32)
                            
    return q   
    
## for convenience I keep the angles between +60 and -60 . anything more or less will be +60 or -60
# also I convert the floating point format into integer.
def saturation(a):
    if a>=(scale/2):
        a=scale/2
    elif a<=-(scale/2):
        a=-(scale/2)
    else:
        a=a
    
    return int(a)
            
L = label()
## delete text from serial print or remove them
start=int(time.time())

while True:
    while arduino_data.inWaiting() == 0:
        pass
    data_packet = arduino_data.readline()
    try:
        paddle.color=paddleColor
        L.visible=False
        
        data_packet = str(data_packet, 'utf-8')
        split_packet = data_packet.split(",")
        # quaternion
        w= float(split_packet[0])
        x= float(split_packet[1])
        y= float(split_packet[2])
        z= float(split_packet[3])
        
        # kalman quaternion
        KalmanW= float(split_packet[4])
        KalmanX= float(split_packet[5])
        KalmanY= float(split_packet[6])
        KalmanZ= float(split_packet[7])
        
        # roll , pitch , yaw
        roll= float(split_packet[8])
        pitch= float(split_packet[9])
        yaw= float(split_packet[10])
        
        #kalman roll , pitch , yaw
        kalmanRoll= float(split_packet[11])
        kalmanPitch= float(split_packet[12])
        kalmanYaw= float(split_packet[13])
        
        ## comp.1 roll , pitch , yaw
        comp1_roll= float(split_packet[14])
        comp1_pitch= float(split_packet[15])
        comp1_yaw= float(split_packet[16])

        q_quat=Q_quaternion(KalmanW,KalmanX,KalmanY,KalmanZ)
        q_rpy=Q_rpy(kalmanRoll*toRad , kalmanPitch*toRad , kalmanYaw*toRad)
        
        if( int(time.time())-start % interval ==0):
            print('                ')
            print("Q(r,p,y) = ")
            print(q_rpy)
            print('                ')
            print("Q(w,x,y,z) = ")
            print(q_quat)
            print('                ')
            print("********************")
        
       

        # Change the attributes of your object to syncronize it with real time motions        
        xx=saturation(kalmanRoll)+(scale/2)
        yy=saturation(kalmanPitch)+(scale/2)
        
        padX=(roomX/scale)*xx -roomX/2
        padY=(roomY/scale)*yy+roomY/2
            
        marbleX=marbleX+deltaX
        marbleY=marbleY+deltaY
        marbleZ=marbleZ+deltaZ
        
        
            
        if marbleX+marbleR>(roomX/2-wallT/2) or marbleX-marbleR<(-roomX/2+wallT/2):
            deltaX=deltaX*(-1)
            marbleX=marbleX+deltaX
     
        if marbleY+marbleR>(roomY/2-wallT/2) or marbleY-marbleR<(-roomY/2+wallT/2):
            deltaY=deltaY*(-1)
            marbleY=marbleY+deltaY
     
        if marbleZ-marbleR<(-roomZ/2+wallT/2):
            deltaZ=deltaZ*(-1)
            marbleZ=marbleZ+deltaZ
            
            
        
        
        if  marbleZ+marbleR>(roomZ/2) :
            if marbleX>padX-paddleX/2 and marbleX<padX+paddleX/2 and marbleY>padY-paddleY/2 and marbleY<padY+paddleY/2 :
               deltaZ=deltaZ*(-1)
               marbleZ=marbleZ+deltaZ
               paddle.color=paddleHit
               time.sleep(0.2)
               
             
            else:
                
                L.text= "GAME OVER! TRY AGAIN :)"
                L.visible=True
                time.sleep(1)
                L.text= "3"
                L.visible=True
                time.sleep(.5)
                L.text= "2"
                L.visible=True
                time.sleep(.5)
                L.text= "1"
                L.visible=True
                time.sleep(.5)
                L.text= "NOW!"
                L.visible=True
                time.sleep(.5)
                L.visible=False
                
                # back to initial location and choosing a random direction
                n=[-1,1]
                marbleX=0
                deltaX=(random.choice(n))*0.03
                 
                marbleY=0
                deltaY=(random.choice(n))*0.03
                 
                marbleZ=0
                deltaZ=.03
                
                padX=paddleX
                padY=paddleY
                padZ=paddleZ
                
                
                
            
        
        marble.pos=vector(marbleX,marbleY,marbleZ)
        paddle.pos=vector(padX,padY,roomZ/2)

                
    except:
        pass
    
            
    