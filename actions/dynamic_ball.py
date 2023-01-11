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

CATCH_SIZE = 35

def search_ball(robot):
    found = False
    w = 1.5 * (2*robot.last_seen_left-1)     # if left, left turn
    while not found:
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame=frame)
        if blob:
            found = True
            logging.info('I\'ve found the ball!')
        else:
            robot.setSpeed(0, w)
    return True

def approach_ball(robot):
    ready = False
    while not ready:
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame=frame)
        if blob:
            frame_size = [frame[1].size, frame[:,1].size] # width x height
            print(frame_size)
            robot.ball_left = (blob.pt[0] < frame_size[0]/2)  # True if the ball is on the left
            if blob.size >= 35:
                logging.info('I\'m close enough, lowering the claw!')
                ready = True
            else:
                v = math.sqrt((35 - blob.size)) / 35 * 1.15 # sqrt used for smoothness
                w = (math.sqrt(abs(frame_size[0]/2 - blob.pt[0]) / (frame_size[0]/2)) * math.pi/4) * (2*robot.ball_left-1) *0.5  # positive or negative value of w
                robot.setSpeed(v, w)
        else:
            logging.warning('I lost the ball :(')
            return False
    return True

def catch_ball(robot):  # FIXME: lo guapo sería poder sacar el vector de v a partir de la cámara (aproximar la posición en la base en cada frame), de momento voy a tirar con un control en bucle cerrado que intente ir a la misma v
    """Tries to catch the ball while moving"""
    ready = False
    robot.BP.set_motor_position(robot.claw_motor, robot.op_cl)
    while not ready:
        frame = robot.takePic()
        blob = helpers.vision.get_blob_new(frame=frame)
        if blob:
            frame_size = [frame[1].size, frame[:,1].size] # width x height
            robot.ball_left = (blob.pt[0] < frame_size[0]/2)  # True if the ball is on the left
            size_difference = CATCH_SIZE - blob.size    # positive if the blob is smaller (needs to go faster)
            v = robot.v.value + size_difference/10
            w = (math.sqrt(abs(frame_size[0]/2 - blob.pt[0]) / (frame_size[0]/2)) * math.pi/4) * (2*robot.ball_left-1)
            robot.setSpeed(v, w)
        else:
            logging.warning('I lost the ball :(')
            return False
    return True

def go_for_ball(robot):
    state = 0
    while state < 3:
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
        elif state == 2:    # cogiendo el peloto
            success = catch_ball(robot)
            if success:
                state = 3
            else:
                state = 2