#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function
from __future__ import division
from asyncio import wait_for
import math
import time     # import the time library for the sleep function
import sage
import numpy as np
import math
import logging

##################
# BASIC MOVEMENTS#
##################
def spin(robot, w, t):
    robot.setSpeed(0, w)
    time.sleep(t)
    robot.setSpeed(0, 0)

def run(robot, v, t):
    robot.setSpeed(v, 0)
    time.sleep(t)
    robot.setSpeed(0, 0)

def abrupt_stop(robot):
    """
    Quickly stops the robot (please, use this for emergencies only!)
    """
    robot.setSpeed(0, 0)
    logging.warning('Abruptly stopped the robot!')

def soft_stop(robot, t = 0.5):
    """
    Steadily stops the robot in the given time
    """
    logging.info('Stopping the robot...')
    v, w = robot.readSpeed()
    delay = t / 10
    for i in range(9, 0):
        robot.setSpeed(v*i/10, w*i/10)
        time.sleep(delay)

####################
# COMPLEX MOVEMENTS#
####################
def eight(robot, r = 0.2, v = 0.2):
    """
    Does an odometry-based eight circuit with the given r and v.
    """
    w = v / r
    logging.info('PRIMER VALOR: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < math.pi/2):
        robot.setSpeed(0, w)
    logging.info('spin: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(v, -w)
    logging.info('primer cacho: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < math.pi/2):
        robot.setSpeed(v, w)
    logging.info('empieza media vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(v, w)
    logging.info('tres cuartos vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < -math.pi/2):
        robot.setSpeed(v,w)
    logging.info('final vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < 0):
        robot.setSpeed(v, -w)
    logging.info('casi acaba el ocho: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > math.pi/2):
        robot.setSpeed(v, -w)
    logging.info('yyy acabamos: {}'.format(robot.th.value/math.pi))
    robot.setSpeed(0, 0)

def slalom(robot, r1=0.1, r2=0.2, d=1, v=0.2):
    """
    Does a odometry-based slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
    #robot.logger.info('Starting an odometry-based slalom.\n Initial position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))
    w1 = v / r1
    w2 = v / r2
    th = (np.pi/2) - math.acos((r2-r1)/d)
    th1 = (np.pi/2 - th)    # angle to turn on the first circle (twice: while leaving and when arriving)
    th2 = 2*(np.pi/2 + th)
    o = math.sqrt(math.pow(d, 2)+math.pow(r2-r1, 2))  # distance to be covered in between circles
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(0, -w1)
        print('mediavuelta')
    while(robot.th.value < th1):
        robot.setSpeed(v, w1)
        print('primer chercolo')
    while(not sage.is_near(robot,np.array([r1*(1-math.tan(th))+d/math.cos(th), r2-r1*(1-math.cos(th))]),0.05)):
        robot.setSpeed(v,0)
        print('amo to recto')
    robot.setSpeed(0,0)    

################
# MESSY THINGS #
################
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
    """
    Does a time-based eight circuit with the given r and v.
    """
    w = v / r
    spin(robot, -math.pi/2, 1)  # 90 degree turn
    robot.setSpeed(v, w)
    time.sleep(math.pi/w)
    robot.setSpeed(v, -w)
    time.sleep(math.pi*2/w)
    robot.setSpeed(v, w)
    time.sleep(math.pi/w)
    robot.stopRobot()
    robot.logger.info('Ended the time-based eight.\n Final position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))

def time_slalom(robot, r1=0.1, r2=0.2, d=1, v=0.2):
    """
    Does a time-based slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
    robot.logger.info('Starting a time-based slalom.\n Initial position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))
    w1 = v / r1
    w2 = v / r2
    th = (np.pi/2) - math.acos((r2-r1)/d)
    th1 = (np.pi/2 - th)    # angle to turn on the first circle (twice: while leaving and when arriving)
    th2 = 2*(np.pi/2 + th)
    o = math.sqrt(math.pow(d, 2)+math.pow(r2-r1, 2))  # distance to be covered in between circles
    spin(robot, -math.pi/2, 1)  # 90 degree turn
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
    robot.logger.info('Ended the time-based slalom.\n Final position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))
