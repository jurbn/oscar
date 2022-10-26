#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from distutils.debug import DEBUG

import sage
import brickpi3 # import the BrickPi3 drivers

import time     # import the time library for the sleep function
import sys
import math
import numpy as np
import csv
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
        self.op_cl = 0 
        self.cl_cl = 0

        self.BP = brickpi3.BrickPi3()
        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_C

        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1)
        self.w_max = 100
        self.v_max = 0.5

        self.lock_odometry = Lock()
        self.odometry_period = 0.05

        self.odometry_file = 'odometry/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.csv'


    def setSpeed(self, v, w):
        """ Sets the speed of both motors to achieve the given speed parameters (angular speed must be in rad/s)
        (Positive w values turn left, negative ones turn right) """
        rps_left = (v - (w * self.length) / 2) / self.radius 
        rps_right = (v + (w * self.length) / 2) / self.radius

        self.BP.set_motor_dps(self.left_motor, math.degrees(rps_left    ))  # BP works on degrees, so we have to transform it :/
        self.BP.set_motor_dps(self.right_motor, math.degrees(rps_right))

    def readSpeed(self):
        """ Returns Oscar's linear (m/s) and angular (rad/s) speed """
        try:
            speed_left = math.radians(self.BP.get_motor_status(self.left_motor)[3]) * self.radius       # get_motor_status returns an array, the 4th element is its dps
            speed_right = math.radians(self.BP.get_motor_status(self.right_motor)[3]) * self.radius     # get_motor_status works in dps, and we need it to be rps
        except Exception:
            print("There was an error while reading the speed of the motors")
            return
        
        w = (speed_right - speed_left) / self.length
        try:
            r = (self.length / 2) * (speed_left + speed_right) / (speed_right - speed_left)
            v = r * w
        except Exception:
            v = speed_left  # si salta algun error es pq R es infinita, en ese caso speed_right = speed_left --> v = speed_left = speed_right
        return v, w


    def closeClaw(self):
        """Closes Oscar's claw and pulls it up"""
        self.BP.set_motor_position(self.claw_motor, self.cl_cl)

    def openClaw(self):
        """Pulls Oscar's claw down and opens it"""
        self.BP.set_motor_position(self.claw_motor, self.op_cl)  
    
    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value


    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.process = Process(target=self.updateOdometry, args=())
        self.process.start()
        logging.info("Odometry was started, PID: {}".format(self.process.pid))

    def updateOdometry(self):
        """
        Updates the location values of the robot and writes them to a .csv file
        """
        with open(self.odometry_file, 'a', newline='') as csvfile:  # first, we write the headers of the csv
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(['x', 'y', 'th'])
                logging.info('holaquetal {},{},{}'.format(self.x.value, self.y.value, math.degrees(self.th.value)))
        [enc_l_1, enc_r_1] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
        time.sleep(0.05)
        [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
        v_l = math.radians((enc_l_2 - enc_l_1) / 0.05) * self.radius
        v_r = math.radians((enc_r_2 - enc_r_1) / 0.05) * self.radius
        w = (v_r - v_l) / self.length
        try:
            r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
            v = r * w
        except Exception:
            v = v_l
        while not self.finished.value:
            tIni = time.clock()
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            w = (v_r - v_l) / self.radius
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

            with open(self.odometry_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([self.x.value, self.y.value, self.th.value])
            logging.info('{}, {}, {}'.format(self.x.value, self.y.value, self.th.value))
            tEnd = time.clock()
            time.sleep(self.odometry_period - (tEnd-tIni))
        
    def stopOdometry(self):
        """
        Must be called when a stop on odometry is desired.
        """
        logging.info('Odometry was stopped')
        self.finished.value = True


    def setup(self):
        """
        Sets Oscar ready to fight: sets limits, detects claw position, checks the camera, etc (maybe a lil brake check / spin check would be okay too?)
        """
        logging.info('Setting up the robot!')
        self.BP.reset_motor_encoder(self.left_motor + self.right_motor)
        self.BP.set_motor_limits(self.claw_motor, 50, 60)
        self.op_cl = self.BP.get_motor_encoder(self.claw_motor)
        self.cl_cl = self.op_cl - 225
        self.startOdometry()


