#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from distutils.debug import DEBUG

import sage
import brickpi3 # import the BrickPi3 drivers
import movement as mv


import time     # import the time library for the sleep function
import sys
import math
import numpy as np
import csv
import cv2 as cv
import logging
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        self.radius = 0.028
        self.length = 0.15
        self.op_cl = 275
        self.cl_cl = 0

        self.BP = brickpi3.BrickPi3()
        self.BP.reset_all()

        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_C
        self.gyro = self.BP.PORT_2
        self.ultrasonic = self.BP.PORT_1
        self.cam = cv.VideoCapture(0)

        self.BP.set_sensor_type(self.gyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.BP.set_sensor_type(self.ultrasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
    
        self.BP.offset_motor_encoder(self.claw_motor, self.BP.get_motor_encoder(self.claw_motor))
        self.BP.set_motor_limits(self.claw_motor, 100, 200)

        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.x2 = Value('d',0.0)
        self.y2 = Value('d',0.0)
        self.th2 = Value('d',0.0)

        self.finished = Value('b',1)
        self.w_max = 100
        self.v_max = 0.5

        self.lock_odometry = Lock()
        self.odometry_period = 0.05
        self.blob_period = 0.5 #!!!!!!!!!!!!!!! inntentar reducirlo jej

        self.odometry_file = 'odometry/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.csv'

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


    def setSpeed(self, v, w):
        """Sets the speed of both motors to achieve the given speed parameters (angular speed must be in rad/s)
        (Positive w values turn left, negative ones turn right)"""
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
            print("There was an error while reading the speed of the motors")
        return v, w_rad
    
    def takePic(self, save: str = None):    # seteamos que ser un string con default value None (Null o whatever en java)
        """Takes a nice picture and stores it as the save parameter"""
        ret = False
        while not ret:
            ret, frame = self.cam.read()
        return frame
    
    def calibrateClaw(self):
        """Calibrates the claw and sets its offset"""
        located = False
        while not located:
            tIni = time.clock()
            frame = self.takePic()[0:640, 240:480]
            blob = sage.get_blob(frame=frame, color='yellow')
            if blob:
                located = True
            tEnd = time.clock()
            time.sleep(blob_period-tEnd+tIni)
        self.BP.reset_motor_encoder(self.claw_motor)

    def searchBall(self, last_pos = None):
        """Uses the camera to locate the ball"""
        found = False
        while not found:
            #tIni = time.clock()
            frame = self.takePic()
            blob = sage.get_blob(frame = frame)
            if blob:
                found = True
                logging.info('Found the ball! Approaching...')
            else:
                self.setSpeed(0, 0.6)   # gira relativamente rápido para simplemente localizarla (SI TENEMOS LAST_POS, GIRAR HACIA AHI!!!!!!!)
            #tEnd = time.clock()
            #time.sleep(self.blob_period-tEnd+tIni)
        return True
    
    def approachBall(self, last_pos = None):
        """The robot will try to get close enough to get the ball, trying to get it centered if possible (not necessary as 
        it's goint to spin on the grabBall function)"""
        ready = False
        while not ready:
            #tIni = time.clock()
            frame = self.takePic()
            blob = sage.get_blob(frame = frame)
            if blob:
                if blob.size >= 93:   #por ejemplo
                    ready = True
                    logging.info('Close enough to the ball, size is: {}'.format(blob.size))
                else:
                    v = ((100 - blob.size) / 100) * 0.2
                    w = ((320 - blob.pt[0]) / 320) * (math.pi/6)
                    self.setSpeed(v, w)
            else:
                logging.warning('Lost the ball! Searching again...')
                return False
            #tEnd = time.clock()
            #time.sleep(self.blob_period-tEnd+tIni)
        return True
    
    def centerBall(self):
       # self.pauseProximity()
        logging.info('Centering the ball...')
        centered = False
        while not centered:
            #tIni = time.clock()
            frame = self.takePic()
            blob = sage.get_blob(frame = frame)
            logging.info('The ball\'s x position is: {}'.format(blob.pt[0]))
            if blob:
                if blob.pt[0] > 325: #un poco mas de la mitad
                    self.setSpeed(0, -0.1)
                elif blob.pt[0] < 315:   # un poco menos de la mitad
                    self.setSpeed(0, 0.1)
                else:
                    self.setSpeed(0, 0)
                    centered = True
                    logging.info('Ball centered and ready to be catched!')
            else:   # si no ve el blob, hace otra foto
                frame = self.takePic()
                blob = sage.get_blob(frame = frame)
                if not blob:
                    return False
            #tEnd = time.clock()
            #time.sleep(self.blob_period-tEnd+tIni)
        return True
 

    def grabBall(self):
        """Once the robot is near the ball, it reorients itself and tries to grab it."""
        distance_array = []
        for i in range(10):
            tIni = time.clock()
            valid = False
            while not valid:
                try:
                    data = self.BP.get_sensor(self.ultrasonic)
                except Exception as error:
                    logging.error(error)
                else:
                    print(data)
                    valid = data < 50
                #time.sleep(self.odometry_period-tEnd+tIni)
            distance_array.append(data)
            tEnd = time.clock()
            time.sleep(self.odometry_period-tEnd+tIni)
        distance = np.median(distance_array) / 100 - 0.08 # lo dividimos para 100 pq las unidades del sensor son cm Y LE METEMOS OFFSET DE 5CM
        if distance > 30:
            return False
        logging.info('The distance to be covered is: {} meters'.format(distance))
        self.BP.set_motor_position(self.claw_motor, self.op_cl)
        point = sage.absolute_offset(self, distance)
        while not sage.is_near(self, point, 0.01):
            time.sleep(0.01)
            self.setSpeed(0.05, 0)
        self.setSpeed(0, 0)
        self.BP.set_motor_position(self.claw_motor, self.cl_cl)
        ####### CHECK IF BALL IN CLAW AND IF NOT RETURN FALSE
        return True

    def goForBall(self):
        """Searches, and goes for the ball"""
        state = 0
        while state < 4:
            self.setSpeed(0, 0)
            if state == 0:      # buscando el peloto
                success = self.searchBall()
                if success:
                    state = 1
            elif state == 1:    # acercandose al peloto 
                success = self.approachBall()
                if success:
                    state = 2
                else:
                    state = 0
            elif state == 2:    # centrando el peloto
                success = self.centerBall()
                if success:
                    state = 3
                else:
                    state = 0                
            elif state == 3:    # cogiendo el peloto
                success = self.grabBall()
                if success:
                    state = 4
                else:
                    state = 2
    def readOdometry(self):
        """Returns current value of odometry estimation"""
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """This starts a new process/thread that will be updating the odometry periodically"""
        self.finished.value = False
        self.process = Process(target=self.updateOdometry, args=())
        self.process2 = Process(target=self.updateOdometry2, args=())
        self.process.start()
        self.process2.start()
        logging.info("Odometry was started, PID: {}:".format(self.process.pid))

    def updateOdometry(self):
        """Updates the location values of the robot and writes them to a .csv file"""
        #with open(self.odometry_file, 'a', newline='') as csvfile:  # first, we write the headers of the csv
            #writer = csv.writer(csvfile, delimiter=',')
            #writer.writerow(['x', 'y', 'th'])
        #[enc_l_1, enc_r_1] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]        
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            tIni = time.clock()
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]

            w = (v_r - v_l) / self.length
            try:
                r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
                v = r * w
            except Exception:
                v = v_l
            s = v*self.odometry_period
            th = w*self.odometry_period

            self.lock_odometry.acquire()
            self.x.value += s * math.cos((self.th.value + th/2))
            self.y.value += s * math.sin((self.th.value + th/2))
            self.th.value = sage.norm_pi(self.th.value + th)
            self.lock_odometry.release()

            #with open(self.odometry_file, 'a', newline='') as csvfile:
            #    writer = csv.writer(csvfile, delimiter=',')
            #    writer.writerow([self.x.value, self.y.value, self.th.value])
            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)
   
    def updateOdometry2(self):
        """Updates the location values of the robot using the gyroscope"""
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            th = sage.norm_pi(math.radians(self.BP.get_sensor(self.gyro)[0]))
            tIni = time.clock()
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            
            # w = math.radians(self.BP.get_sensor(self.gyro)[1])
            w = math.radians(self.BP.get_sensor(self.gyro)[1])
            try:
                r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
                v = r * w
            except Exception:
                v = v_l
            s = v*self.odometry_period

            self.lock_odometry.acquire()
            self.x2.value += s * math.cos((self.th2.value + th/2))
            self.y2.value += s * math.sin((self.th2.value + th/2))
            self.th2.value = th
            self.lock_odometry.release()

            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)
        
    def stopOdometry(self):
        """Must be called when a stop on odometry is desired"""
        logging.info('Odometry was stopped')
        self.finished.value = True
