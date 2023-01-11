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

def calibrate_claw(robot): 
    """Calibrates the claw and sets its offset"""
    located = False
    while not located:
        tIni = time.clock()
        frame = robot.takePic()[0:640, 240:480]
        blob = helpers.vision.get_blob_new(frame=frame, color='yellow')
        if blob:
            located = True
        tEnd = time.clock()
        time.sleep(robot.blob_period-tEnd+tIni)
    robot.BP.reset_motor_encoder(robot.claw_motor)

def search_ball(robot):
    """Uses the camera to locate the ball"""
    found = False
    if  robot.last_seen_left:
        w = 1
    else: w = -1
    robot.reduction = 0.25
    while not found:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame = frame)
        if blob:
            found = True
            logging.info('Found the ball! Approaching...')
        else:
            robot.setSpeed(0, w)   # gira relativamente rápido para simplemente localizarla (SI TENEMOS LAST_POS, GIRAR HACIA AHI!!!!!!!)
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True

def approach_ball(robot, last_pos = None):
    """The robot will try to get close enough to get the ball, trying to get it centered if possible (not necessary as 
    it's goint to spin on the grabBall function)"""
    ready = False
    robot.reduction = 0.25
    while not ready:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame = frame)
        if blob:
            if blob.pt[0] > (640/2)*robot.reduction: #derecha
                robot.last_seen_left = False
            elif blob.pt[0] < (640/2)*robot.reduction:   #izquierda
                robot.last_seen_left = True
            if blob.size >= 30:
                ready = True
                logging.info('Close enough to the ball, size is: {}'.format(blob.size))
            else:
                v = ((40 - blob.size) / 40) * 0.25
                w = ((640/2)*robot.reduction - blob.pt[0]) / ((640/2)*robot.reduction) * (math.pi/3)
                robot.setSpeed(v, w)
        else:
            logging.warning('Lost the ball! Searching again...')
            return False
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True

def center_ball(robot):
    # robot.pauseProximity()
    logging.info('Centering the ball...')
    centered = False
    robot.reduction = 0.25
    while not centered:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame = frame)
        if blob:
            logging.info('The ball\'s x position is: {}'.format(blob.pt[0]))
            if blob.pt[0] > (640/2)*robot.reduction + 10: #un poco mas de la mitad
                robot.setSpeed(0, -0.4)
                robot.last_seen_left = False
            elif blob.pt[0] < (640/2)*robot.reduction - 10:   # un poco menos de la mitad
                robot.setSpeed(0, 0.4)
                robot.last_seen_left = True
            else:
                robot.setSpeed(0, 0)
                centered = True
                logging.info('Ball centered and ready to be catched!')
        else:   # si no ve el blob, hace otra foto
            frame = robot.takePic()
            blob = helpers.vision.get_blob_new(frame = frame)
            if not blob:
                logging.warning('Lost the ball! Searching agan...')
                return False
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True


def grab_ball(robot):
    """Once the robot is near the ball, it reorients itrobot and tries to grab it."""
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
                print(data)
                valid = True
            time.sleep(robot.odometry_period-time.clock()+tIni)
        distance_array.append(data)
        tEnd = time.clock()
        time.sleep(robot.odometry_period-tEnd+tIni)
    distance = np.median(distance_array) / 100 - 0.1# lo dividimos para 100 pq las unidades del sensor son cm Y LE METEMOS OFFSET DE 5CM
    if distance > 0.30:
        return False
    logging.info('The distance to be covered is: {} meters'.format(distance))
    robot.BP.set_motor_position(robot.claw_motor, robot.op_cl)
    time.sleep(0.5)
    #point = sage.absolute_offset(robot, distance)
    #while not sage.is_near(robot, point, 0.01):
    robot.setSpeed(0.1, 0)
    time.sleep(distance/0.1 + 0.12)
    robot.setSpeed(0, 0)
    robot.BP.set_motor_position(robot.claw_motor, robot.cl_cl)
    time.sleep(0.6)
    ####### CHECK IF BALL IN CLAW AND IF NOT RETURN FALSE
    return True

def go_for_ball(robot):
    """Searches, and goes for the ball"""
    state = 0
    while state < 4:
        robot.setSpeed(0, 0)
        if state == 0:      # buscando el peloto
            success = search_ball(robot)
            if success:
                state = 1
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
                state = 2

def ball_caught(robot):
    """Checks wether or not oscar got the ball"""
    frame = robot.takePic()[219:239, 139:179]
    rows, cols, _ = frame.shape
    rowsA, colsA = 0,0

    print ('tamaño imagen: {}x{} pixels comprobados: {}x{} (origen en centro inferior: ({}, 0))'.format(rows,cols,rowsA,colsA,cols/2))

    return 'holi soy ballCaught y estoy incompleta'