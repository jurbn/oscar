#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from distutils.debug import DEBUG


import libs.brickpi3 as brickpi3 # import the BrickPi3 drivers

import helpers.map
import helpers.maths

import time
import math
import csv
import cv2 as cv
import numpy as np
import logging
import picamera
from picamera.array import PiRGBArray
from multiprocessing import Process, Value, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], side = None):
        """
        Initialize basic robot params. \
        Initialize Motors and Sensors according to the set up in your robot
        """
        init_pos_A = np.array([0.6, 2.6, -math.pi/2])
        init_pos_B = np.array([2.2, 2.6, -math.pi/2])
        if side is 'A':
            self.x = Value('d', init_pos_A[0])
            self.y = Value('d', init_pos_A[1])
            self.th = Value('d', init_pos_A[2])
            self.map_file = 'res/maps/mapaA_CARRERA2020.txt'
        else:
            self.x = Value('d', init_pos_B[0])
            self.y = Value('d', init_pos_B[1])
            self.th = Value('d', init_pos_B[2])
            self.map_file = 'res/maps/mapaB_CARRERA2020.txt'
        
        self.offset = [self.x.value, self.y.value, self.th.value]
        self.objective = [0, 0, 0]
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        self.finished = Value('b',1)
        self.w_max = 100
        self.v_max = 0.5

        [self.map_size, self.map] = helpers.map.read_map(self.map_file)
        self.grid = helpers.map.generate_grid(self.map, self.objective)
        self.grid_plot = self.grid  # plottable grid (auxiliar)
        self.black = True           # True if our map is BLACK

        # Construction params #
        self.radius = 0.028
        self.length = 0.134
        self.op_cl = 275
        self.cl_cl = 0

        # Actuators and sensors #
        self.BP = brickpi3.BrickPi3()

        self.left_motor = self.BP.PORT_C
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_A

        self.gyro = self.BP.PORT_2
        self.frontasonic = self.BP.PORT_4
        self.laterasonic = self.BP.PORT_3
        self.light = self.BP.PORT_1

        self.BP.reset_all()
        
        self.cam = cv.VideoCapture(0)
        self.reduction = 0.25

        self.BP.set_sensor_type(self.gyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.BP.set_sensor_type(self.frontasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.BP.set_sensor_type(self.light, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        self.BP.offset_motor_encoder(self.claw_motor, self.BP.get_motor_encoder(self.claw_motor))
        self.BP.offset_motor_encoder(self.left_motor, self.BP.get_motor_encoder(self.left_motor))
        self.BP.offset_motor_encoder(self.right_motor, self.BP.get_motor_encoder(self.right_motor))  
        self.BP.set_motor_limits(self.claw_motor, 100, 400)

        # other internal params #
        self.lock_odometry = Lock()
        self.odometry_period = 0.05
        self.blob_period = 0.5 

        self.ball_caught_in = None
        self.last_seen_left = False
        self.changed = False

        self.odometry_file = 'logs/odometry/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.csv'
        
        logging.info('ROBOT: Robot set up!')
        error = True
        while error:    # we loop until we can use the gyro sensor
            time.sleep(0.1)
            try:
                self.BP.get_sensor(self.gyro)
            except Exception:
                pass
            else:
                error = False
        self.startOdometry()

    def forceNewPosition(self, new_pos):
        """
        Forces a new xy position for the Robot (teleport)
        """
        logging.info('ROBOT forceNewPosition: Updating position to {}'.format(new_pos))
        self.lock_odometry.acquire()
        self.x.value = new_pos[0]
        self.y.value = new_pos[1]
        try:
            self.offset_angle = new_pos[2] - (self.th.value - self.offset_angle)
        except Exception:
            pass
        self.lock_odometry.release()
    
    def updateWithMapFile(self, file):
        """
        Updates the map and grid params given a map file
        """
        try:
            self.map_file = file
            [self.map_size, self.map] = helpers.map.read_map(self.map_file)
            self.grid = helpers.map.generate_grid(self.map, self.objective)
            self.grid_plot = self.grid
        except Exception:
            logging.error('An error occurred while updating the map!')

    def setMapByColor(self, black = False):
        """
        Sets a map file (and therefore its params and grid) given the color of the first tile
        """
        if black:
            logging.info('ROBOT setMapByColor: Tile color is black, setting map B')
            self.forceNewPosition([2.2, 3])
            self.last_seen_left = True
            self.updateWithMapFile('res/maps/mapaB_CARRERA2020.txt')
            self.objective = [6, 3] 
        else:
            logging.info('ROBOT setMapByColor:Tile color is white, setting map A')
            self.forceNewPosition([0.6, 3])
            self.last_seen_left = False
            self.updateWithMapFile('res/maps/mapaA_CARRERA2020.txt')
            self.objective = [2, 3]

    def isFloorBlack(self):
        """Returns True if black, False if white"""
        i = 0
        value_arr = np.array([], dtype='float')
        while i < 5:
            time.sleep(0.02)
            try:
                value = self.BP.get_sensor(self.light)
            except Exception:
                logging.info('One of the light values was not valid')
            else:
                value_arr = np.append(value_arr, value)
                i += 1
        value = np.median(value_arr)
        self.black = value > 2600
        return self.black # if true, black; if false, white
            
    def getFrontsonic(self):
        """Returns the value of the front ultrasonic sensor"""
        error = True
        while error:
            time.sleep(0.03)
            try:
                value = self.BP.get_sensor(self.frontasonic)
            except Exception:
                logging.info('Frontasonic took an invalid value, repeating...')
            else:
                error = False
        return value

    def setSpeed(self, v, w):
        """Sets the speed of both motors to achieve the given speed parameters (angular speed must be in rad/s)
        (Positive w values turn left, negative ones turn right)"""
        self.v.value = v
        self.w.value = w
        rps_left = (v - (w * self.length) / 2) / self.radius 
        rps_right = (v + (w * self.length) / 2) / self.radius
        self.BP.set_motor_dps(self.left_motor, math.degrees(rps_left))  # BP works on degrees, so we have to transform it :/
        self.BP.set_motor_dps(self.right_motor, math.degrees(rps_right))

    def readSpeed(self):
        """Returns Oscar's linear (m/s) and angular (rad/s) speed"""
        try:
            w_deg = self.BP.get_sensor(self.gyro)
            w_rad = math.radians(w_deg)
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
            v = r * w_rad
        except Exception:
            logging.debug("There was an error while reading the speed of the motors")
        return v, w_rad
    
    def takePic(self, PI = False, save: str = None):
        """Takes a nice picture and saves it on the save parameter's path.
        PI parameter specifies wether picam or opencv libraries will be used."""
        if not PI:
            if type(self.cam) is picamera.camera.PiCamera:
                self.cam.close()
                self.cam = cv.VideoCapture(0)
            ret = False
            while not ret:
                ret, frame = self.cam.read()
        else:
            if type(self.cam) is cv.VideoCapture:
                self.cam.release()
                self.cam = picamera.PiCamera()
            self.cam.resolution = (640, 480)
            self.cam.framerate = 10
            rawCapture = PiRGBArray(self.cam)
            self.cam.capture(rawCapture, format="bgr")
            frame = rawCapture.array
            rawCapture.truncate(0)
        frame  = cv.rotate(frame, cv.ROTATE_180)
        frame = cv.resize(frame, None, fx = self.reduction, fy = self.reduction, interpolation = cv.INTER_LANCZOS4)
        return frame


    def readOdometry(self):
        """Returns current value of odometry estimation"""
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """This starts a new process/thread that will be updating the odometry periodically"""
        self.finished.value = False
        self.process = Process(target=self.updateOdometry, args=())
        self.process.start()
        logging.info("ROBOT: Odometry was started, PID: {}:".format(self.process.pid))
   
    def updateOdometry(self):
        """Updates the location values of the robot using the gyroscope"""
        with open(self.odometry_file, 'a', newline='') as csvfile:  # first, we write the headers of the csv
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(['x', 'y', 'th'])
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            tIni = time.clock()
            try:
                gyro_values = self.BP.get_sensor(self.gyro)
            except Exception:
                gyro_values = [self.th.value, self.w.value]
            th = helpers.maths.norm_pi(self.offset[2] - math.radians(gyro_values[0])) # se le mete el offset al giroscopio pq el cabron lo reinicia
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            v = (v_l + v_r)/2
            s = v*self.odometry_period
            self.lock_odometry.acquire()    # we lock the odometry values
            th_calc = helpers.maths.norm_pi(self.th.value+(th-self.th.value)/2)
            self.x.value += s * math.cos(th_calc)
            self.y.value += s * math.sin(th_calc)
            self.th.value = th
            self.lock_odometry.release()    # we release those values
            with open(self.odometry_file, 'a', newline='') as csvfile:  # write odometry values on the .csv file
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([self.x.value, self.y.value, self.th.value])
            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)  # wait until the period is completed
        logging.info('Odometry was stopped')

        
    def stopOdometry(self):
        """Must be called when a stop on odometry is desired"""
        self.finished.value = True
