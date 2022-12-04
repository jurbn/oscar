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
def spin(robot, th, w = 1.5):
    """Makes Oscar turn a specified angle (in radians)"""
    th = sage.norm_pi(th) 
    while not sage.is_near_angle(robot, th):
        robot.setSpeed(0, th/abs(th)*w)
    robot.setSpeed(0, 0)

def run(robot, objctv, v = 1.5):
    """Makes Oscar go straight forward to the specified position (in meters)"""
    while not sage.is_near(robot, objctv): #molaría añadir si eso una condicion por tiempo o delta de pos para asegurar que llega
        robot.setSpeed(v, 0)
    robot.setSpeed(0, 0)

def arc(robot, objctv, v = 1.5):
    """Makes Oscar advance in a circular motion to the specified location (in meters)"""
    R = (pow(objctv[0],2) + pow(objctv[1],2))/(2*objctv[1])
    w = v/R
    while not (sage.is_near(robot, objctv) or sage.is_near_angle(robot, w/abs(w)*math.pi()/2)):
        robot.setSpeed(v, w)
    robot.SetSpeed(0,0)

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

def go_to_cell(robot, map, move, goal, clockwise):   #TODO: meter los movimientos y tal
    """moves the robot given the goal array position being:\n
        moves:          relative goals:\n
        7   0   1       [-1,-1]  [0,-1]  [1,-1]\n
        6   x   2       [-1,0]      x    [1,0]\n
        5   4   3       [-1,1]    [0,1]   [1,1]\n
    with x facing up(0) and y facing left(6)"""
    goal = arr2pos(goal)
    if move == 0: 
        print('voy recto')
        run(robot, goal)
    elif (move == 1) and not clockwise:
        print('arco a la derecha')
        arc(robot, goal)
    elif move == 1:
        print('giro derecha y arco a la izquierda')
        spin(robot, -math.pi()/2)
        arc(robot, goal)
    elif move == 2:
        print('giro derecha y voy recto')
        spin(robot, -math.pi()/2)
        run(robot, goal)
    elif (move == 3) and not clockwise:
        print('giro derecha y arco a la derecha')
        spin(robot, -math.pi()/2)
        arc(robot, goal)
    elif move == 3:
        print('giro 180 y arco a la izquierda')
        spin(robot, math.pi())
        arc(robot, goal)
    elif move == 4:
        print('giro 180 y voy recto')
        spin(robot, math.pi())
        run(robot, goal)
    elif (move == 5) and not clockwise:
        print('giro 180 y giro derecha')
        spin(robot, math.pi())
        arc(robot, goal)
    elif move == 5:
        print('giro izquierda y arco a la izquierda')
        spin(robot, math.pi()/2)
        arc(robot, goal)
    elif move == 6: 
        print('giro izquierda y voy recto')
        spin(robot, math.pi()/2)
        run(robot, goal)
    elif (move == 7) and clockwise:
        print('arco a la izquierda')
        arc(robot,goal)
    elif move == 7:
        print('giro izquierda y arco a la derecha')
        spin(robot, math.pi()/2)
        arc(robot,goal)

def go_to_useless(robot, pos, v = 0.1):
    """Moves the robot from its original position to the given one drawing an arc"""
    og_pos = [robot.x.value, robot.y.value, robot.th.value]
    if len(pos) == 2:
        x = pos[0]-og_pos[0]    # x2-x1
        y = pos[1]-og_pos[1]    # y2-y1
        direction = math.atan2(y, x)    # angulo del vector (oscar-pos) con x
        angle = sage.norm_pi(robot.th.value - direction)  # diferencia entre donde mira oscar y donde mira el vector que los une
        if sage.is_near_angle(og_pos[2], direction, 3*math.pi/8): # AÑADIR la condicion de que la casilla justo de enfrente esté libre: se pueden hacer esos arcos.
            print('la siguiente casilla está enfrente')
        else: # si no está en frente, prueba a girar 90º en la direccion que marque el angulo 'direction'
            print('me voy a recolocar')
            while (abs(robot.th.value - og_pos[2]) < math.pi/2): #mientras haya girado (en abs) menos de pi/2:
                robot.setSpeed(0, (math.pi/4) * (angle/abs(angle))) #gira
                 
            if sage.is_near_angle(robot.th.value - og_pos[2], direction, 3*math.pi/8): # está a su izda o en la celda de debajo (ahora enfrente o enfrente-izq)
                print('la siguiente casilla estaba a la izda/dcha, ahora enfrente')       
            else: # la casilla estaba detrás nuestro todo este tiempo 
                while abs(robot.th.value - og_pos[2]) < math.pi:
                    robot.setSpeed(0, (math.pi/4) * (angle/abs(angle)))
                print('la siguiente casilla estaba detrás, ahora enfrente')
            #elif sage.is_near_angle(og_pos[2]-math.pi, direction, math.pi/8):# si está detrás
             #   print('la siguiente casilla está detrás')
             #   destiny = sage.norm_pi(og_pos[2]+math.pi)
             #   while sage.norm_pi(robot.th.value-destiny)>0.05:
             #       robot.setSpeed(0, math.pi/4)
                #while robot.th.value < math.pi: # ESTO DEBERIA SER QUE SE DE MEDIA VUELTA
                #    robot.setSpeed(0, -math.pi/4)
        robot.setSpeed(0, 0) #una vezz rrecolocado tiene que parar jej
        print('I should be facing  {}, and im at {}'.format(direction, robot.th.value))
        
        th_f = robot.th.value + np.arctan2(2*x*y, pow(x,2)-pow(y,2)) #formula
        #now the trayectory, either straight forward or arc movement facing forward
        if (abs(y) < 0.1):    # SI NO TIENE QUE GIRAR
            print('turborecto')
            w = 0
        else:           # SI TIENE QUE GIRAR
            print('no tan turborecto')
            R = (pow(x,2) + pow(y,2))/(2*y)
            w = v/R
        print('la velocidad angular es: ',w)
        finished_move = False
        while not finished_move:
            if robot.BP.get_sensor(robot.ultrasonic) < 20: # si esta cerca de pared, se para y devuelve false (obstaculo)
                print('AAAAA UN OBSTACULO ME ASUSTE')
                robot.setSpeed(0, 0)
                return False
            else:
                robot.setSpeed(v, w)
            time.sleep(0.05)
            finished_move = (sage.is_near(robot, pos, threshold=0.01)) or (sage.is_near_angle(robot.th.value, th_f, 0.25))
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
    while (robot.location != obj):
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

