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

###################
# BASIC MOVEMENTS #
###################
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

def go_to(robot, pos, v = 0.1):
    """Moves the robot from its original position to the given one drawing an arc"""
    og_pos = [robot.x.value, robot.y.value, robot.th.value]
    print('original: {}'.format(og_pos))
    if len(pos) == 2:
        x = pos[0]-og_pos[0]    # x2-x1
        y = pos[1]-og_pos[1]    # y2-y1
        print('distance is {}, {}'.format(x, y))
        direction = math.atan2(y, x)    # angulo del vector (oscar-pos) con x
        angle = sage.norm_pi(robot.th.value - direction)  # diferencia entre donde mira oscar y donde mira el vector que los une
        if -3*math.pi/8 <= angle <= 3*math.pi/8: # si está en frente
            print('to recto')
        else:   # si no está en frente, SE GIRA PARA MIRAR AL DESTINO
            if angle > 7*math.pi/8 or angle < -7*math.pi/8:# si está detrás
                print('atrás')
                destiny = sage.norm_pi(og_pos[2]+math.pi)
                while sage.norm_pi(robot.th.value-destiny)>0.05:
                    robot.setSpeed(0, math.pi/4)
                #while robot.th.value < math.pi: # ESTO DEBERIA SER QUE SE DE MEDIA VUELTA
                #    robot.setSpeed(0, -math.pi/4)
            elif 3*math.pi/8 < angle <= 7*math.pi/8: # está a su izda
                print('izda')
                destiny = sage.norm_pi(og_pos[2]+math.pi/2)
                while sage.norm_pi(robot.th.value-destiny)>0.05:
                    robot.setSpeed(0, math.pi/4)
            elif -3*math.pi/8 > angle >= -7*math.pi/8:
                print('dcha')
                destiny = sage.norm_pi(og_pos[2]-math.pi/2)
                while sage.norm_pi(robot.th.value-destiny)>0.05:
                    robot.setSpeed(0, -math.pi/4)
        robot.setSpeed(0, 0)
        print('I should be {}, and im {}'.format(direction, robot.th.value))
        th_f = robot.th.value + np.arctan2(2*x*y, pow(x,2)-pow(y,2))
        # YA ESTA MIRANDO AL DESTINO
        if (y == 0):    # SI NO TIENE QUE GIRAR
                w = 0
        else:           # SI TIENE QUE GIRAR
            R = (pow(x,2) + pow(y,2))/(2*y)
            w = v/R
        while not sage.is_near(robot, pos, threshold=0.08):
            if robot.BP.get_sensor(robot.ultrasonic) < 20: # si esta cerca de pared, se para y devuelve false (obstaculo)
                print('AAAAA UN OBSTACULO ME ASUSTE')
                robot.setSpeed(0, 0)
                return False
            else:
                robot.setSpeed(v, w)
            time.sleep(0.01)
        robot.setSpeed(0, 0)
    return True
    #elif len(pos) == 3: #si la th.final calculada en el arco no coincide nos acercamos a un punto aproximado (habrá que ver si aproximamos en x o y o los dos),
        # luego giramos en el punto aproximado y vamos en linea recta al destino o lo q sea
        # VALE NO SERIA COS y SEN del th deseado para sacar el pto de aproximacion sino no se puede ir en linea recta al final 


#####################
# COMPLEX MOVEMENTS =0.4):
#####################
def square (robot, l=0.4):
    w = 1
    v = 0.1
    while(robot.th.value < math.pi/2):
        robot.setSpeed(0, w)
    logging.info('primer giro. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value < l/2):
        robot.setSpeed(v, 0)
    logging.info('primera recta. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    logging.info('segundo giro. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.x.value < l):
        robot.setSpeed(v, 0)
    logging.info('segunda recta. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(0, -w)
    logging.info('tercer giro. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value > -l/2):
        robot.setSpeed(v, 0)
    logging.info('tercera recta. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value < 0):
        robot.setSpeed(0, -w)
    logging.info('cuarto giro. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.x.value > 0):
        robot.setSpeed(v, 0)
    logging.info('cuarta recta. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > math.pi/2):
        robot.setSpeed(0, -w)
    logging.info('ultimo giro. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value < 0):
        robot.setSpeed(v, 0)
    logging.info('ultima recta. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    logging.info('y ya estaría. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))


def eight(robot, r = 0.2, v = 0.1):
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
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    robot.setSpeed(0, 0)

def slalom(robot, r1=0.1, r2=0.2, d=0.5, v=0.1):
    """
    Does a odometry-based slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
    #robot.logger.info('Starting an odometry-based slalom.\n Initial position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))
    logging.info('Starting the slalom...')
    w1 = v / r1
    w2 = v / r2
    th = math.pi/2 - abs(math.acos((r2-r1)/d)) # we abs it so we have a first cuadrant angle
    print(th)
    tx1 = r1-r1*math.sin(th)
    ty1 = -r1*math.cos(th)
    tx2 = r1+d-r2*math.sin(th)
    ty2 = -r2*math.cos(th)
    t11 = np.array([tx1, ty1])
    t12 = np.array([tx1, -ty2])
    t21 = np.array([tx2, ty2])
    t22 = np.array([tx2, -ty2])

    logging.info('Spinning at: {}, {}, {}'.format(robot.x.value, robot.y.value, robot.th.value))
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(0, -w2)
    logging.info('primer tramo! {}, {}, {}'.format(robot.x.value, robot.y.value, robot.th.value))
    while(robot.th.value <  -th):
        robot.setSpeed(v, w1)
    logging.info('amo to recto {}, {}, {}'.format(robot.x.value, robot.y.value, robot.th.value))
    while((not sage.is_near(robot, t21, 0.01)) and robot.x.value < tx2):
        logging.debug('Desired position: {}\nCurrent position: {}, {}'.format(t21, robot.x.value, robot.y.value))
        robot.setSpeed(v,0)
    logging.info('y giramo')
    while(robot.th.value < 0):
        robot.setSpeed(v, w2)
    while(robot.th.value > 0):
        robot.setSpeed(v, w2)
    while(robot.th.value < (-math.pi + th)):
        robot.setSpeed(v, w2)
    logging.info('giro terminao, to recto')
    while((not sage.is_near(robot, t12, 0.01)) and robot.x.value > tx1):
        logging.debug('Desired position: {}\nCurrent position: {}, {}'.format(t12, robot.x.value, robot.y.value))
        robot.setSpeed(v,0)
    logging.info('empezamo el ultimo giro')
    while(robot.th.value < -math.pi/2): # vamos de -pi a -pi/2
        robot.setSpeed(v, w1)
    logging.info('Finished the circuit!!!')
    while(robot.th.value < 0):
        robot.setSpeed(0, w2)
    robot.setSpeed(0,0)    

################
# MESSY THINGS #
################

def enc_test (robot):
    print('encoder 3 = {}'.format(robot.BP.fet_motot_encoder(robot.claw_motor))) 

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

