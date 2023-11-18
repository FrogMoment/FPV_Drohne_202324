from vpython import *
from time import *
import numpy as np
import math

scene.range = 10
toRad = 2 * np.pi / 360
toDeg = 1 / toRad
scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600

xArrow = arrow(length=2, shaftwidth=.3, color=color.red,   axis=vector(1, 0, 0))
yArrow = arrow(length=2, shaftwidth=.3, color=color.green, axis=vector(0, 1, 0))
zArrow = arrow(length=2, shaftwidth=.3, color=color.blue,  axis=vector(0, 0, 1))

frontArrow = arrow(length=8, shaftwidth=.3, color=color.cyan,    axis=vector(1, 0, 0))
upArrow =    arrow(length=8, shaftwidth=.3, color=color.magenta, axis=vector(0, 1, 0))
sideArrow =  arrow(length=8, shaftwidth=.3, color=color.yellow,  axis=vector(0, 0, 1))

IMUBoard = box(length=6, width=6, height=6, opacity=.8, pos=vector(0,0,0))

while True:
    pitch = 15 * toRad

    for yaw in np.arange(0, 2*np.pi, .01):
        rate(50)
        k = vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))

        y = vector(0, 1, 0) # always up

        s = cross(k, y)

        v = cross(s, k)

        frontArrow.axis = k
        frontArrow.length = 6
        IMUBoard.axis = k
        IMUBoard.length = 6
        IMUBoard.width = 6
        IMUBoard.height = 6

        sideArrow.axis = s
        sideArrow.length = 6
        
        upArrow.axis = v
        IMUBoard.up = v
        upArrow.length = 6


