#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from asyncio import wait_for
import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import math
import sage
import numpy as np

def spinn (robot):
    robot.setSpeed(0,100)
    time.sleep(5)
    robot.setSpeed(0,0)

def runn (robot):
    robot.setSpeed(100,0)
    time.sleep(5)
    robot.setSpeed(0,0)

def spin (robot):
    robot.setSpeed(0,60)
    time.sleep(5)
    robot.setSpeed(0,0)

def run (robot):
    robot.setSpeed(60,0)
    time.sleep(5)
    robot.setSpeed(0,0)

def moveC (robot,pos,R,th, t=0):
    #voy a hacer el control con la funcion del cálculo de la posición tras el movimiento y los datos de odometría.
    if t==0:
        w=robot.w_max*0.75
        v=w*R
        t=th*R/v
        obj=sage.pos_bot([v,w],pos,t)
        robot.setSpeed(v,w)
    else:
        w=th/t
        v=w*R
        obj=sage.pos_bot([v,w],pos,t)
        robot.setSpeed(v,w)
    while(robot.readOdometry!=obj):
            time.sleep(0.1)

def eight(robot, r, w):
    v = w*r
    robot.setSpeed(v, w)
    print('a')
    time.sleep(np.pi*2/w)
    robot.setSpeed(v, -w)
    print('e')
    time.sleep(np.pi*2/w)
    robot.stopRobot()

