#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division
from asyncio import wait_for
import math
import brickpi3  # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import sage
import numpy as np
import math
import logging

def spin(robot, w, t):
    robot.setSpeed(0, w)
    time.sleep(t)
    robot.setSpeed(0, 0)


def run(robot, v, t):
    robot.setSpeed(v, 0)
    time.sleep(t)
    robot.setSpeed(0, 0)


def moveC(robot, pos, R, th, t=0):
    # voy a hacer el control con la funcion del cálculo de la posición tras el movimiento y los datos de odometría.
    if t == 0:
        w = robot.w_max*0.75
        v = w*R
        t = th*R/v
        obj = sage.pos_bot([v, w], pos, t)
        robot.setSpeed(v, w)
    else:
        w = th/t
        v = w*R
        obj = sage.pos_bot([v, w], pos, t)
        robot.setSpeed(v, w)
    while (robot.readOdometry != obj):
        time.sleep(0.1)

def time_eight(robot, r = 0.2, v = 0.2):
    w = v / r
    spin(robot, -math.pi/2, 1)
    robot.setSpeed(v, w)
    time.sleep(math.pi/w)
    robot.setSpeed(v, -w)
    time.sleep(math.pi*2/w)
    robot.setSpeed(v, w)
    time.sleep(math.pi/w)
    robot.stopRobot()


def time_slalom(robot, r1=0.1, r2=0.2, d=1, v=0.2):
    """
    Does a slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
    w1 = v / r1
    w2 = v / r2
    th = (np.pi/2) - math.acos((r2-r1)/d)
    th1 = (np.pi/2 - th)    # angle to turn on the first circle (twice: while leaving and when arriving)
    th2 = 2*(np.pi/2 + th)
    o = math.sqrt(math.pow(d, 2)+math.pow(r2-r1, 2))  # distance to be covered in between circles
    robot.setSpeed(v, w1)
    time.sleep(th1/w1)
    robot.setSpeed(v, 0)
    time.sleep(o/v)
    robot.setSpeed(v, w2)
    time.sleep(th2/w2)
    robot.setSpeed(v, 0)
    time.sleep(o/v)
    robot.setSpeed(v, w1)
    time.sleep(th1/w1)
    robot.stopRobot()

def abruptStop(robot):
        """
        Quickly stops the robot
        """
        robot.setSpeed(0, 0)
        robot.log_logger.warning('Stopped the robot!')  

def softStop(robot):
    """
    Steadily stops the robot
    """
    v, w = robot.readSpeed()
    robot.log_logger.info('Stopping the robot...')
    for i in range(0.9, 0, 0.1):
        robot.setSpeed(v*i, w*i)
        time.sleep(0.05)
    robot.log_logger.info('The robot was stopped!')


