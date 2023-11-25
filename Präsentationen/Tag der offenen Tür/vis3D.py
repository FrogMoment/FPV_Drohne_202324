from vpython import *
from time import *
import numpy as np
import math
import serial

scene.range = 10
toRad = 2 * np.pi / 360
toDeg = 1 / toRad
scene.forward = vector(-1, -1, -1)

scene.width = 1500
scene.height = 900

xArrow = arrow(length=2, shaftwidth=.3, color=color.red,   axis=vector(1, 0, 0))
yArrow = arrow(length=2, shaftwidth=.3, color=color.green, axis=vector(0, 1, 0))
zArrow = arrow(length=2, shaftwidth=.3, color=color.blue,  axis=vector(0, 0, 1))

frontArrow = arrow(length=8, shaftwidth=.3, color=color.cyan,    axis=vector(1, 0, 0))
upArrow =    arrow(length=8, shaftwidth=.3, color=color.magenta, axis=vector(0, 1, 0))
sideArrow =  arrow(length=8, shaftwidth=.3, color=color.yellow,  axis=vector(0, 0, 1))

IMUBoard = box(length=6, width=6, height=6, opacity=.8, pos=vector(0,0,0))
myObj = compound([IMUBoard])

with serial.Serial('COM7', baudrate=115200) as ser: 
    while True:
        while(ser.inWaiting() == 0):
            pass
        text = str(ser.readline())
        text = text.split(",")

        pitch = float(text[0][2:]) * toRad
        roll = float(text[1][:-6]) * toRad
        yaw = 0

        print("Roll=",roll*toDeg," Pitch=",pitch*toDeg,"Yaw=",yaw*toDeg)
        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)
    
        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=8
        frontArrow.length=8
        upArrow.length=8