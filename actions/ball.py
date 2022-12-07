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

def calibrateClaw(robot):
    """Calibrates the claw and sets its offset"""
    located = False
    while not located:
        tIni = time.clock()
        frame = robot.takePic()[0:640, 240:480]
        blob = helpers.vision.get_blob(frame=frame, color='yellow')
        if blob:
            located = True
        tEnd = time.clock()
        time.sleep(robot.blob_period-tEnd+tIni)
    robot.BP.reset_motor_encoder(robot.claw_motor)

def searchBall(robot):
    """Uses the camera to locate the ball"""
    found = False
    if  robot.last_seen_left:
        w = 1.5
    else: w = -1.5
    while not found:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = helpers.vision.get_blob(frame = frame)
        if blob:
            found = True
            logging.info('Found the ball! Approaching...')
        else:
            robot.setSpeed(0, w)   # gira relativamente rápido para simplemente localizarla (SI TENEMOS LAST_POS, GIRAR HACIA AHI!!!!!!!)
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True

def approachBall(robot, last_pos = None):
    """The robot will try to get close enough to get the ball, trying to get it centered if possible (not necessary as 
    it's goint to spin on the grabBall function)"""
    ready = False
    while not ready:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = helpers.vision.get_blob(frame = frame)
        if blob:
            if blob.pt[0] > 160: #derecha
                robot.last_seen_left = False
            elif blob.pt[0] < 160:   #izquierda
                robot.last_seen_left = True
            if blob.size >= 60:   #por ejemplo
                ready = True
                logging.info('Close enough to the ball, size is: {}'.format(blob.size))
            else:
                v = ((72.5 - blob.size) / 72.5) * 0.25
                w = ((160 - blob.pt[0]) / 160) * (math.pi/6)
                robot.setSpeed(v, w)
        else:
            logging.warning('Lost the ball! Searching again...')
            return False
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True

def centerBall(robot):
    # robot.pauseProximity()
    logging.info('Centering the ball...')
    centered = False
    while not centered:
        #tIni = time.clock()
        frame = robot.takePic()
        blob = sage.get_blob(frame = frame)
        if blob:
            logging.info('The ball\'s x position is: {}'.format(blob.pt[0]))
            if blob.pt[0] > 162: #un poco mas de la mitad
                robot.setSpeed(0, -0.1)
                robot.last_seen_left = False
            elif blob.pt[0] < 158:   # un poco menos de la mitad
                robot.setSpeed(0, 0.1)
                robot.last_seen_left = True
            else:
                robot.setSpeed(0, 0)
                centered = True
                logging.info('Ball centered and ready to be catched!')
        else:   # si no ve el blob, hace otra foto
            frame = robot.takePic()
            blob = helpers.vision.get_blob(frame = frame)
            if not blob:
                return False
        #tEnd = time.clock()
        #time.sleep(robot.blob_period-tEnd+tIni)
    return True


def grabBall(robot):
    """Once the robot is near the ball, it reorients itrobot and tries to grab it."""
    distance_array = []
    for i in range(10):
        tIni = time.clock()
        valid = False
        while not valid:
            try:
                data = robot.BP.get_sensor(robot.ultrasonic)
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

def goForBall(robot):
    """Searches, and goes for the ball"""
    state = 0
    while state < 4:
        robot.setSpeed(0, 0)
        if state == 0:      # buscando el peloto
            success = robot.searchBall()
            if success:
                state = 1
        elif state == 1:    # acercandose al peloto 
            success = robot.approachBall()
            if success:
                state = 2
            else:
                state = 0
        elif state == 2:    # centrando el peloto
            success = robot.centerBall()
            if success:
                state = 3
            else:
                state = 0                
        elif state == 3:    # cogiendo el peloto
            success = robot.grabBall()
            if success:
                state = 4
            else:
                state = 2

def ballCaught(robot):
    """Checks wether or not oscar got the ball"""
    frame = robot.takePic()[219:239, 139:179]
    rows, cols, _ = frame.shape
    rowsA, colsA = 0,0

    print ('tamaño imagen: {}x{} pixels comprobados: {}x{} (origen en centro inferior: ({}, 0))'.format(rows,cols,rowsA,colsA,cols/2))

    return 'holi soy ballCaught y estoy incompleta'