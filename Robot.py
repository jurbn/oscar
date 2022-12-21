#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from distutils.debug import DEBUG


import libs.brickpi3 as brickpi3 # import the BrickPi3 drivers

import helpers.map
import helpers.maths

import time     # import the time library for the sleep function
import math
import csv
import cv2 as cv
import numpy as np
import logging
from multiprocessing import Process, Value, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])

        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        self.finished = Value('b',1)
        self.w_max = 100
        self.v_max = 0.5
        
        self.offset = init_position

        self.map_file = 'maps/mapa3.txt'
        self.cell = [0, 0]
        [self.map_size, self.map] = helpers.map.read_map(self.map_file)

        self.radius = 0.028
        self.length = 0.15
        self.op_cl = 275
        self.cl_cl = 0

        self.BP = brickpi3.BrickPi3()

        self.left_motor = self.BP.PORT_C
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_A

        self.gyro = self.BP.PORT_2
        self.frontasonic = self.BP.PORT_1
        self.laterasonic = self.BP.PORT_4
        self.light = self.BP.PORT_3

        self.BP.reset_all()
        
        self.cam = cv.VideoCapture(0)
        self.reduction = 0.5

        self.BP.set_sensor_type(self.gyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.BP.set_sensor_type(self.frontasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.BP.set_sensor_type(self.light, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        self.BP.offset_motor_encoder(self.claw_motor, self.BP.get_motor_encoder(self.claw_motor))
        self.BP.offset_motor_encoder(self.left_motor, self.BP.get_motor_encoder(self.left_motor))
        self.BP.offset_motor_encoder(self.right_motor, self.BP.get_motor_encoder(self.right_motor))  
        self.BP.set_motor_limits(self.claw_motor, 100, 400)

        self.lock_odometry = Lock()
        self.odometry_period = 0.05
        self.blob_period = 0.5 
        self.last_seen_left = False
        self.changed = False

        self.odometry_file = 'logs/odometry/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.csv'
        
        logging.info('Robot set up!')
        error = True
        while error:
            time.sleep(0.1)
            try:
                self.BP.get_sensor(self.gyro)
            except Exception:
                pass
            else:
                error = False
        self.startOdometry()

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
        value = np.median(value_arr)
        return value > 2700 # if true, black; if false, white
            

    def getFrontsonic(self):
        """Returns the value of the frontsonic"""
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

    def startTeabag(self):
        self.finish_tb.value = False
        self.teabag = Process(target=self.updateTeabag, args=())
        self.teabag.start()
        logging.info('Teabag started, PID: {}'.format(self.teabag.pid))

    def updateTeabag(self):
        while not self.finish_tb.value:
            tIni = time.clock()
            try:
                distance = self.getFrontsonic()
                if distance < 15:
                    pass
            except Exception:
                pass
            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)

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
    
    def takePic(self, save: str = None):    # seteamos que ser un string con default value None (Null o whatever en java)
        """Takes a nice picture and stores it as the save parameter"""
        ret = False
        while not ret:
            ret, frame = self.cam.read()
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
        logging.info("Odometry was started, PID: {}:".format(self.process.pid))
   
    def updateOdometry(self):
        """Updates the location values of the robot using the gyroscope"""
        with open(self.odometry_file, 'a', newline='') as csvfile:  # first, we write the headers of the csv
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(['x', 'y', 'th'])
        #[enc_l_1, enc_r_1] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]      TODO: hemo comentado  
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            tIni = time.clock()
            #if self.changed:
            #    self.setSpeed(self.v.value, 0)
            th = helpers.maths.norm_pi(self.offset[2] - math.radians(self.BP.get_sensor(self.gyro)[0])) # se le mete el offset al giroscopio pq el cabron lo reinicia
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            
            w = -math.radians(self.BP.get_sensor(self.gyro)[1])
            #if (self.w.value == 0) and (w != 0):
            #    self.changed = True
            #    self.setSpeed(self.v.value, w)
            # try:
            #     r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
            #     v = r * w
            # except Exception:
            #     v = v_r
            v = (v_l + v_r)/2
            #if self.v.value == 0:  TODO: comentao
            #    v = 0
            s = v*self.odometry_period

            self.lock_odometry.acquire()
            th_calc = helpers.maths.norm_pi(self.th.value+(th-self.th.value)/2)
            self.x.value += s * math.cos(th_calc)    #s * math.cos(th)*2
            self.y.value += s * math.sin(th_calc)    #s * math.sin(th)*2
            self.th.value = th
            self.lock_odometry.release()
            with open(self.odometry_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([self.x.value, self.y.value, self.th.value])
            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)
        logging.info('Odometry was stopped')

        
    def stopOdometry(self):
        """Must be called when a stop on odometry is desired"""
        self.finished.value = True
