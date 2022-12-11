from asyncio import wait_for
import math
import time     # import the time library for the sleep function
import numpy as np
import math
import logging
import sys
import os

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import helpers.location
import helpers.maths
import helpers.simulation

###################
# BASIC MOVEMENTS #
###################
def spin(robot, th, w = 1.5):
    """Makes Oscar turn a specified angle (in radians)"""
    th = helpers.maths.norm_pi(th + robot.th.value) 
    w = (2*(th >= 0)-1)*w
    while not helpers.location.is_near_angle(robot, th):
        robot.setSpeed(0, w)
    robot.setSpeed(0, 0)

def run(robot, objctv, v = 0.1):
    """Makes Oscar go straight forward to the specified position (in meters)"""
    th = robot.th.value
    # te los comento pq a mi me funciona el run y no los necesito! <3
    #print('IM IN: {}'.format(math.sqrt(pow(robot.x.value * math.cos(th), 2) + pow(robot.y.value*math.sin(th), 2))))
    #print('I NEED: {}'.format(math.sqrt(pow(objctv[0]*math.cos(th), 2) + pow(objctv[1]*math.sin(th), 2))))
    #print('abs resta: ', abs(math.sqrt(pow(robot.x.value * math.cos(th), 2) + pow(robot.y.value*math.sin(th), 2)) - math.sqrt(pow(objctv[0]*math.cos(th), 2) + pow(objctv[1]*math.sin(th), 2))) > 0.02)
    #print('is near: ',helpers.location.is_near(robot, objctv, threshold=0.02))
    while (not helpers.location.is_near(robot, objctv, threshold=0.02)) and (abs(math.sqrt(pow(robot.x.value * math.cos(th), 2) + pow(robot.y.value*math.sin(th), 2)) - math.sqrt(pow(objctv[0]*math.cos(th), 2) + pow(objctv[1]*math.sin(th), 2))) > 0.02): #molaría añadir si eso una condicion por tiempo o delta de pos para asegurar que llega
        near = robot.getFrontsonic() < 25
        if near:
            raise Exception('OH NOOO A WALL <:o')
        robot.setSpeed(v, 0)
        #print('está: {} quiere llegar a: {}, is_near: {}'.format([robot.x.value, robot.y.value], objctv, helpers.location.is_near(robot, objctv, threshold=0.02)))
    robot.setSpeed(0, 0)

def arc(robot, objctv, v = 0.1, clockwise = True):
    """Makes Oscar advance in a circular motion to the specified location (in meters)"""
    objctv = np.array(objctv)
    mvmnt = objctv - [robot.x.value, robot.y.value]
    logging.debug('el vector es: {}'.format(mvmnt))
    R = (pow(mvmnt[0],2) + pow(mvmnt[1],2))/(2*mvmnt[1]) * (2*clockwise-1)  # TODO:OJO QUE R DEPENDE DE Y, aunque nos tenemos que fijar en el signo de x a veces tambien???
    logging.debug('LA R ES: {}'.format(R))
    w = v/R 
    th = helpers.maths.norm_pi((2*(w >= 0)-1)*math.pi/2 + robot.th.value) 
    while not (helpers.location.is_near(robot, mvmnt) or helpers.location.is_near_angle(robot, th)):
        near = robot.getFrontsonic() < 25
        if near:
            robot.setSpeed(0,0)
            raise Exception('OH NOOO A WALL >:o')
        robot.setSpeed(v, w)
    robot.setSpeed(0,0)

#TODO: def arc(robot, goal) funcion que calcule arcos en funcion del destino O del radio especificado

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


#####################
# COMPLEX MOVEMENTS #
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

def slalom_right(robot, map_size, v = 0.5): 
    """
    Slaloms around the columns ehen on the A map (white tile)
    """
    try:
        tile_size = map_size[2]
    except Exception:
        tile_size = map_size
    #estamos en el lado izquierdo
    origin = [0.6, 3, -math.pi/2] #origin_A  DEBERIA SER -PI/2 PERO LO CAMBIO PARA PROBAR  
    R = tile_size/math.sqrt(2) 
    logging.debug('voy a girar')
    #logging.debug('mi pos es: {}\n voy a girar hasta: {}\n'.format([robot.x.value,robot.y.value,robot.th.value],[robot.x.value,robot.y.value, origin[2] + math.pi/4 * ('B' in map)*2-1]))
    spin(robot, - math.pi/4)
    logging.debug('he girado, voy recto')
    #logging.debug('mi pos es: {}\n voy a girar hasta: {}\n'.format([robot.x.value,robot.y.value,robot.th.value],[origin[0] + tile_size/2 * (('B' in map)*2-1), origin[1]- 3*tile_size/2, robot.th.value]))
    run(robot, [0.4, 1.6])#[origin[0] - tile_size/2, origin[1]- 3*tile_size/2])
    logging.debug('he ido recto, voy a hacer un arco')
    arc(robot, [origin[0] - tile_size/2, origin[1]- 3*tile_size/2])
    logging.debug('he hecho un arco, voy recto')
    run(robot, [origin[0] + tile_size/2, origin[1]- 5*tile_size/2])
    logging.debug('he ido recto, voy a hacer un arco')
    arc(robot, [origin[0] + tile_size/2, origin[1]- 7*tile_size/2])
    logging.debug('he hecho un arco, recolocacion final')
    l = (math.sqrt(pow(tile_size, 2) + pow(tile_size, 2)/4))
    t = l/v
    w = (math.pi/4)/t
    while not helpers.location.is_near(robot, [origen[0], origen[1]+ 11*tile_size/2], 0.05):
        robot.setSpeed(v, w)
    robot.setSpeed(0, 0)
    logging.debug('he acabado el slalom')




def slalom_useless(robot, map_size, map, v = 0.5): #TODO: arreglarlo o quitarlo antes de entregar <3
    """
    Slaloms around the columns depending on the given map (A or B)
    """
    try:
        tile_size = map_size[2]
    except Exception:
        tile_size = map_size
    #estamos en el lado derecho del mapa da positivo, sino negativo
    if 'B' in map:
        origin = [0.2, 1.8, math.pi] #origin_B
        logging.debug('al menos sé leer')
    else:
        origin = [0.6, 3, -math.pi/2] #origin_A    
    
    R = tile_size/math.sqrt(2) 
    logging.debug('voy a girar')
    logging.debug('mi pos es: {}\n voy a girar hasta: {}\n'.format([robot.x.value,robot.y.value,robot.th.value],[robot.x.value,robot.y.value, origin[2] + math.pi/4 * ('B' in map)*2-1]))
    spin(robot, origin[2] + math.pi/4 * (('B' in map)*2-1))
    logging.debug('he girado, voy recto')
    logging.debug('mi pos es: {}\n voy a girar hasta: {}\n'.format([robot.x.value,robot.y.value,robot.th.value],[origin[0] + tile_size/2 * (('B' in map)*2-1), origin[1]- 3*tile_size/2, robot.th.value]))
    run(robot, [origin[0] + tile_size/2 * (('B' in map)*2-1), origin[1]- 3*tile_size/2])
    logging.debug('he ido recto, voy a hacer un arco')
    arc(robot, [origin[0] + tile_size/2 * (('B' in map)*2-1), origin[1]- 3*tile_size/2])
    logging.debug('he hecho un arco, voy recto')
    run(robot, [origin[0] - tile_size/2 * (('B' in map)*2-1), origin[1]- 5*tile_size/2])
    logging.debug('he ido recto, voy a hacer un arco')
    arc(robot, [origin[0] - tile_size/2 * (('B' in map)*2-1), origin[1]- 7*tile_size/2])
    logging.debug('he hecho un arco, recolocacion final')
    while not helpers.location.is_near(robot, [origen[0], origen[1]+ 11*tile_size/2], 0.05):
        robot.setSpeed(v, robot. math.pi/2 * (('B' in map)*2-1)/((math.sqrt(pow(tile_size, 2) + pow(tile_size, 2)/4))/v))
    robot.setSpeed(0, 0)
    logging.debug('he acabado el slalom')


   

def tronchopocho(robot, r1=0.1, r2=0.2, d=0.5, v=0.1):
    """
    Does a odometry-based slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
    #robot.logger.info('Starting an odometry-based slalom.\n Initial position: ({}, {}, {})'.format(robot.x.value, robot.y.value, robot.th.value))
    logging.info('Starting the slalom...')
    w1 = v / r1
    w2 = v / r2
    th = math.pi/2 - abs(math.acos((r2-r1)/d)) # we abs it so we have a first cuadrant angle
    logging.debug(th)
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
    while((not helpers.location.is_near(robot, t21, 0.01)) and robot.x.value < tx2):
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
    while((not helpers.location.is_near(robot, t12, 0.01)) and robot.x.value > tx1):
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
    logging.debug('encoder 3 = {}'.format(robot.BP.fet_motot_encoder(robot.claw_motor))) 

def moveC(robot, pos, R, th, t=0):
    # voy a hacer el control con la funcion del cálculo de la posición tras el movimiento y los datos de odometría.
    if t == 0:
        w = robot.w_max*0.75
        v = w*R
        t = th*R/v
        obj = helpers.simulation.pos_bot([v, w], pos, t)
        robot.setSpeed(v, w)
    else:
        w = th/t
        v = w*R
        obj = helpers.simulation.pos_bot([v, w], pos, t)
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

