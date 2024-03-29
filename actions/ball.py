import time
import logging
import math
import sys
import os
import numpy as np

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import helpers.vision
import actions.moves

def search_ball(robot, center = True):
    """Uses the camera to locate the ball"""
    logging.info('SEARCH_BALL: Searching for the ball...')
    found = False
    if robot.last_seen_left:    # if we've seen the ball before, spin on that direction
        w = 1.5
    else: w = -1.5
    robot.reduction = 0.25  # camera resolution reduction
    while not found:        # search until found
        frame = robot.takePic()
        blob = helpers.vision.get_blob(frame = frame)
        if blob:
            found = True
            logging.info('SEARCH_BALL: Found the ball! Approaching...')
        elif helpers.location.is_near_angle(robot.th.value, math.pi * robot.black) and center:  # if center flag is on and we haven't found it in a while, we go to the CENTER
            return False
        else:
            robot.setSpeed(0, w)
    return True

def approach_ball(robot, last_pos = None):
    """The robot will try to get close enough to get the ball, trying to get it centered if possible (not necessary as 
    it's goint to spin on the grabBall function)"""
    logging.info('APPROACH_BALL: Approaching the ball...')
    ready = False
    robot.reduction = 0.25
    while not ready:
        frame = robot.takePic()
        blob = helpers.vision.get_blob(frame = frame)
        if blob:
            if blob.pt[0] > (640/2)*robot.reduction:    # right
                robot.last_seen_left = False
            elif blob.pt[0] < (640/2)*robot.reduction:  # left
                robot.last_seen_left = True
            if blob.size >= 30:
                ready = True
                logging.info('Close enough to the ball, size is: {}'.format(blob.size))
            else:
                v = ((30 - blob.size) / 20) * 0.35
                w = ((640/2)*robot.reduction - blob.pt[0]) / ((640/2)*robot.reduction) * (math.pi/2.5)
                robot.setSpeed(v, w)
        else:
            logging.warning('Lost the ball! Searching again...')
            return False
    return True

def center_ball(robot):
    """Spins until the ball is on the center of the screen for at least three frames"""
    logging.info('Centering the ball...')
    first = False
    second = False
    centered = False
    robot.reduction = 0.25
    while not centered:
        frame = robot.takePic()
        blob = helpers.vision.get_blob(frame = frame)
        if blob:
            if blob.pt[0] > (640/2)*robot.reduction + 2: #un poco mas de la mitad
                robot.setSpeed(0, -0.35)
                first = False
                second = False
                robot.last_seen_left = False
            elif blob.pt[0] < (640/2)*robot.reduction - 2:   # un poco menos de la mitad
                robot.setSpeed(0, 0.35)
                first = False
                second = False
                robot.last_seen_left = True
            else:
                robot.setSpeed(0, 0)
                centered = second   # centered True if previous two waere centered too
                second = first
                first = True
        else:
            frame = robot.takePic()
            blob = helpers.vision.get_blob(frame = frame)
            if not blob:
                logging.warning('Lost the ball! Searching agan...')
                return False
    logging.info('Ball centered and ready to be catched!')
    return True


def grab_ball(robot, check_caught = False):
    """Once the robot is near the ball, it reorients itrobot and tries to grab it."""
    logging.info('Grabbing the ball...')
    enc_beg = robot.BP.get_motor_encoder(robot.claw_motor)
    distance_array = []
    for i in range(5):
        tIni = time.clock()
        valid = False
        while not valid:
            try:
                data = robot.getFrontsonic()
            except Exception as error:
                logging.error(error)
            else:
                logging.debug(data)
                valid = True
            time.sleep(robot.odometry_period-time.clock()+tIni)
        distance_array.append(data)
        tEnd = time.clock()
        time.sleep(robot.odometry_period-tEnd+tIni)
    distance = np.median(distance_array) / 100 - 0.1    # lo dividimos para 100 pq las unidades del sensor son cm Y LE METEMOS OFFSET DE 5CM
    if distance > 0.30:
        return False
    logging.info('The distance to be covered is: {} meters'.format(distance))
    robot.BP.set_motor_position(robot.claw_motor, robot.op_cl)
    time.sleep(0.5)
    robot.setSpeed(0.1, 0)
    time.sleep(distance/0.1 + 0.12)
    robot.setSpeed(0, 0)
    robot.BP.set_motor_position(robot.claw_motor, robot.cl_cl)
    time.sleep(1.5)
    if check_caught:
        enc_dif = robot.BP.get_motor_encoder(robot.claw_motor) - enc_beg
        logging.debug('BALL grab_ball: encoder value is {}'.format(enc_dif))
        if enc_dif > 1:
            logging.info('Got the ball!')
            return True
        else:
            logging.warning('Didn\'t get the ball :(')
            return False
    else:
        return True

def go_for_ball(robot, center = True):
    """Searches, and goes for the ball"""
    state = 0
    while state < 4:
        robot.setSpeed(0, 0)
        if state == 0:      # buscando el peloto
            success = search_ball(robot, center)
            if success:
                state = 1
            elif not success and center == True:
                center = False  # para que no se repita!
                actions.map.go_to_center(robot)
        elif state == 1:    # acercandose al peloto 
            success = approach_ball(robot)
            if success:
                state = 2
            else:
                state = 0
        elif state == 2:    # centrando el peloto
            success = center_ball(robot)
            if success:
                state = 3
            else:
                state = 0                
        elif state == 3:    # cogiendo el peloto
            success = grab_ball(robot)
            if success:
                state = 4
            else:
                state = 0
    robot.ball_caught_in = [robot.x.value, robot.y.value]

def check_caught(robot, black): # esto es lo de que el robot se mire en un espejo que al final no lo hemos hecho
    """Checks wether or not oscar got the ball"""
    found = False
    actions.moves.spin(robot, math.pi*(not black), relative = False)
    front_value = robot.getFrontsonic()
    initial_value = front_value
    while not (22 < front_value < 28):  # acercarnos para chequiar
        v = 0.015 * (front_value - 25)
        if v > 0.15: v = 0.15
        elif v < -0.15: v = -0.15
        new_front_value = robot.getFrontsonic()
        robot.setSpeed(v, 0)
        if new_front_value > 60:
            front_value = front_value
        else:
            front_value = new_front_value
    frame = robot.takePic()
    blob = helpers.vision.get_blob(frame = frame)
    if blob: found = True
    front_value = robot.getFrontsonic()
    while not (initial_value-2 < front_value < initial_value+2):  # acercarnos para chequiar
        v = 0.015 * (front_value - 20)
        if v > 0.15: v = 0.15
        elif v < -0.15: v = -0.15
        new_front_value = robot.getFrontsonic()
        robot.setSpeed(v, 0)
        if new_front_value > 60:
            front_value = front_value
        else:
            front_value = new_front_value
    return found


    