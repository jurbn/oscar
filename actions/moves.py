from asyncio import wait_for
import math
import time
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

def spin(robot, th, w = 1, relative = True, threshold = 0.03):
    """Makes Oscar turn a specified angle (in radians)"""
    if relative:    # if relative flag is setted, we will spin relatively to the robot's position
        th = helpers.maths.norm_pi(robot.th.value + th)
    else:
        th = helpers.maths.norm_pi(th)
    spin_dir = -helpers.maths.norm_pi(robot.th.value - th)
    w = helpers.maths.get_sign(spin_dir)*w  # specify speed and direction
    while not helpers.location.is_near_angle(robot, th, threshold = threshold):
        robot.setSpeed(0, w)
    robot.setSpeed(0, 0)

def run(robot, objctv, v = 0.2, correct_trajectory = True, detect_obstacles = False, threshold = 0.025):
    """Makes Oscar go straight forward to the specified position (in meters)"""
    th = robot.th.value
    near = False
    if detect_obstacles:
            near = robot.getFrontsonic() < 45
            if near:    # if an obstacle was detected, stop and correct position
                logging.warning('OH NOOO A WALL <:o')
                robot.setSpeed(0, 0)
                front_value = robot.getFrontsonic()
                while not (18 < front_value < 22):
                    v = 0.025 * (front_value - 20)
                    if v > 0.15: v = 0.15
                    elif v < -0.15: v = -0.15
                    new_front_value = robot.getFrontsonic()
                    th = robot.th.value
                    ob_th = helpers.location.get_robot_quadrant(robot)
                    w = helpers.maths.norm_pi(ob_th-th)
                    robot.setSpeed(v, w)
                    if new_front_value > 60:
                        front_value = front_value
                    else:
                        front_value = new_front_value
                robot.setSpeed(0, 0)
                raise Exception('Seen a wall')
    while (not helpers.location.is_near(robot, objctv, threshold=threshold)) and (abs(math.sqrt(pow(robot.x.value * math.cos(th), 2) + pow(robot.y.value*math.sin(th), 2)) - math.sqrt(pow(objctv[0]*math.cos(th), 2) + pow(objctv[1]*math.sin(th), 2))) > threshold):
        if correct_trajectory:
            th = robot.th.value
            ob_th = helpers.location.get_robot_quadrant(robot)
            w = helpers.maths.norm_pi(ob_th-th)
        else:
            w = 0
        robot.setSpeed(v, w)

def arc(robot, objctv, v = 0.15, clockwise = True, detect_obstacles = False):
    """Makes Oscar advance in a circular motion to the specified location (in meters)"""
    objctv = np.array(objctv)
    mvmnt = objctv - [robot.x.value, robot.y.value]
    R = abs((pow(mvmnt[0],2) + pow(mvmnt[1],2))/(2*mvmnt[1]))
    logging.debug('ARC: im in {} and i want to go to {} with radius {}\n'.format([robot.x.value, robot.y.value], objctv, R))
    w = v/R  * -(2*clockwise-1)
    th_end = helpers.maths.norm_pi((2*(w >= 0)-1)*math.pi/2 + robot.th.value) 
    while not (helpers.location.is_near(robot, objctv, threshold=0.015) or helpers.location.is_near_angle(robot, th_end, threshold=0.02)):
        if detect_obstacles:
            near = robot.getFrontsonic() < 22
        else:
            near = False
        if near:
            robot.setSpeed(0,0)
            raise Exception('OH NOOO A WALL >:o')
        robot.setSpeed(v, w)

def abrupt_stop(robot):
    """
    Quickly stops the robot (please, use this for emergencies only!)
    """
    robot.setSpeed(0, 0)
    logging.warning('ABRUPT_STOP: abruptly stopped the robot!')

def soft_stop(robot, t = 0.5):
    """
    Steadily stops the robot in the given time
    """
    logging.info('SOFT_STOP: stopping the robot...')
    v, w = robot.readSpeed()
    delay = t / 10
    for i in range(9, 0):
        robot.setSpeed(v*i/10, w*i/10)
        time.sleep(delay)


#####################
# COMPLEX MOVEMENTS #
#####################
def square (robot, l=0.4):
    """A simple square used for odometry calibration"""
    w = 1
    v = 0.1
    while(robot.th.value < math.pi/2):
        robot.setSpeed(0, w)
    logging.info('SQUARE: first spin done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value < l/2):
        robot.setSpeed(v, 0)
    logging.info('SQUARE: first side done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    logging.info('SQUARE: second spin done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.x.value < l):
        robot.setSpeed(v, 0)
    logging.info('SQUARE: second side done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(0, -w)
    logging.info('SQUARE: third spin done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value > -l/2):
        robot.setSpeed(v, 0)
    logging.info('SQUARE: third side done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value < 0):
        robot.setSpeed(0, -w)
    logging.info('SQUARE: fourth spin done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.x.value > 0):
        robot.setSpeed(v, 0)
    logging.info('SQUARE: fourth side done. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > math.pi/2):
        robot.setSpeed(0, -w)
    logging.info('SQUARE: last spin. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.y.value < 0):
        robot.setSpeed(v, 0)
    logging.info('SQUARE: finished the first side. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    logging.info('SQUARE: done!. Pos: ({}m, {}m, {}pi rad)'.format(robot.x.value, robot.y.value, robot.th.value/math.pi))


def eight(robot, r = 0.2, v = 0.1):
    """
    Does an odometry-based eight circuit with the given r and v.
    """
    w = v / r
    while(robot.th.value < math.pi/2):
        robot.setSpeed(0, w)
    logging.info('EIGHT: spin: {}'.format(robot.th.value))
    while(robot.th.value > -math.pi/2):
        robot.setSpeed(v, -w)
    logging.info('EIGHT: first circle: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < math.pi/2):
        robot.setSpeed(v, w)
    logging.info('EIGHT: empieza media vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(v, w)
    logging.info('EIGHT: tres cuartos vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < -math.pi/2):
        robot.setSpeed(v,w)
    logging.info('EIGHT: final vuelta: {}'.format(robot.th.value/math.pi))
    while(robot.th.value < 0):
        robot.setSpeed(v, -w)
    logging.info('EIGHT: casi acaba el ocho: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > math.pi/2):
        robot.setSpeed(v, -w)
    logging.info('EIGHT: yyy acabamos: {}'.format(robot.th.value/math.pi))
    while(robot.th.value > 0):
        robot.setSpeed(0, -w)
    robot.setSpeed(0, 0)

def half_eight(robot, black, v = 0.25):
    """
    Does an odometry-based half-eight circuit with the given r and v.
    """
    r = 0.325
    th = robot.th.value
    side = black * 2 - 1
    w = v / r
    while not helpers.location.is_near_angle(robot.th.value, th + side * math.pi/2):
        robot.setSpeed(0, side * w * 2)
    while not helpers.location.is_near_angle(robot.th.value, th - side * math.pi/2):
        robot.setSpeed(v, -w * side)
    while not helpers.location.is_near_angle(robot.th.value, th + side * math.pi/2):
        robot.setSpeed(v, w * side)
    #while not helpers.location.is_near_angle(robot.th.value, th):
    #    robot.setSpeed(0, -w * side * 2)
    robot.setSpeed(0, 0)
    logging.info('HALF_EIGHT: done!')

def half_eight_short(robot, black, v = 0.2):
    """
    Does a short odometry-based half-eight circuit with the given r and v. [ No utilizado pq rosario no nos deja :( ]
    """
    r = 0.35
    th = robot.th.value
    side = black * 2 - 1
    w = v / r
    while not helpers.location.is_near_angle(robot.th.value, th + side * math.pi/2):
        robot.setSpeed(0, side * w * 2)
    while not helpers.location.is_near_angle(robot.th.value, th - side * math.pi/2):
        robot.setSpeed(v, -w * side)
    while (not helpers.location.is_near_angle(robot.th.value, th)) and (robot.y.value > 1.4):
        robot.setSpeed(v, w * side)
    logging.info('HALF_EIGHT_SHORT: done!')


def slalom(robot, black = False, map_size = 0.4, v = 0.5): 
    """
    Slaloms around the columns when on the A map (white tile)
    """
    try:
        tile = map_size[2]
    except Exception:
        tile = map_size
    #we get the robot's coordinates and orientation at the start:
    x = robot.x.value 
    y = robot.y.value
    th = robot.th.value #they should be: [0.6, 3, -pi/2] 
    R = tile/math.sqrt(2)
    side = black * 2 - 1#(('B' in robot.map_file) * 2) - 1 #-1 for mapaA(left side, white tile, right slalom) and 1 for mapaB(right side, black tile, left slalom)  
    logging.debug('side: {} (-1 = A, 1 = B)\n'.format(side))
    #logging.debug('SLALOM_0: \nim gonna go straight from {} to {}\n'.format([x,y], [[x, y - tile]]))
    #run(robot, [x, y - tile])
    logging.debug('SLALOM_1: \n(im at {})\nim gonna turn to {}\n'.format([robot.x.value, robot.y.value, robot.th.value], (side * math.pi/4) + th))
    spin(robot, (side * math.pi/4)) #spin usa datos relativos!!
    logging.debug('SLALOM_2: \ni\'ve spun (my th is {})\nimma go straight, from {} to {}\n'.format(robot.th.value, [robot.x.value, robot.y.value], [x + (side * tile/2), y - tile/2]))
    while robot.y.value > (y - tile/2):
        robot.setSpeed(0.2, 0)
    robot.setSpeed(0, 0)    
    logging.debug('SLALOM_3: \ni\'ve runn\nimma do an arc, from {} to {}\n'.format([robot.x.value, robot.y.value, robot.th.value], [ x + (side * tile/2), y - tile * 5/2, th - side * math.pi / 4]))
    arc(robot, [x + (side * tile/2), y - tile * 3/2], clockwise = (side +1)/2, v = 0.2)
    logging.debug('SLALOM_4: \ni\'ve arched (my pos is {})\nimma go straight, from {} to {}\n'.format([robot.x.value, robot.y.value, robot.th.value], [robot.x.value, robot.y.value], [x - side * tile/2, y - tile * 7/2]))
    while robot.y.value > (y - tile * 5/2):
        robot.setSpeed(0.2, 0)
    robot.setSpeed(0, 0)
    logging.debug('SLALOM_5: \ni\'ve run (im at {})\n imma do a final ·.*.\' arc .\'·.* from {} to {}\n'.format([robot.x.value, robot.y.value], robot.th.value, th + side*math.pi/4))
    arc(robot, [x - side * tile/2, y - tile * 7/2], clockwise = not((side +1)/2))
    logging.debug('SLALOM_6: \narc! finished!! (my pos is: {})\nalmost there the objective is {}\n'.format([robot.x.value, robot.y.value, robot.th.value], [x, y - tile * 5]))
    while robot.y.value > (y - tile * 4):
        robot.setSpeed(0.15, 0)
    robot.setSpeed(0, 0)
    logging.debug('SLALOM_7: \nim here!! i cant believe it!!! ಥ‿ಥ (my pos is {})\n now i only gotta spin to {}\n'.format([robot.x.value, robot.y.value, robot.th.value], th))
    spin(robot, (-side * math.pi/4))
    logging.debug('SLALOM_8:\n we... we made it.... u.u')
   

def tronchopocho(robot, r1=0.1, r2=0.2, d=0.5, v=0.1):
    """
    Does a odometry-based slalom with the given r1, r2, diameter and linear speed.\n r1 must be greater than r2.
    """
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

#############################
# MESSY (and unused) THINGS #
#############################

def enc_test (robot):
    logging.debug('encoder 3 = {}'.format(robot.BP.fet_motot_encoder(robot.claw_motor))) 

def moveC(robot, pos, R, th, t=0):
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

