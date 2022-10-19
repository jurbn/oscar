#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

import sage
import brickpi3 # import the BrickPi3 drivers

import time     # import the time library for the sleep function
import sys
import math
import csv
import logging
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.radius = 0.0275
        self.length = 0.145
        self.op_cl = 0 #value of the open claw (encoder)
        self.cl_cl = 0 #value of the closed claw (encoder)
        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_C

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        #self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        #self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.w_max = 100
        self.v_max = 100

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period
        self.odometry_period = 1.0

        self.odometry_file = 'odometry/' + time.strftime('%Y%m%d%H%M%S') + '.csv'
        self.log_file = 'log/' + time.strftime('%Y%m%d%H%M%S') + '.log'

    def setSpeed(self, v, w):
        """ Sets the speed of both motors to achieve the given speed parameters (angular speed must be in rad/s)
        (Positive w values turn left, negative ones turn right) """
        print("setting speed to %.2f m/s and %.2f rad/s" % (v, w))
        rps_left = (v - (w * self.length) / 2) / self.radius 
        rps_right = (v + (w * self.length) / 2) / self.radius
        print('Right rad/s: {}. Left rad/s: {}'.format(rps_left, rps_right))
        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        self.BP.set_motor_dps(self.left_motor, math.degrees(rps_left    ))  # BP works on degrees, so we have to transform it :/
        self.BP.set_motor_dps(self.right_motor, math.degrees(rps_right))

    def close_claw(self):
        """Closes Oscar's claw and pulls it up"""
        self.BP.set_motor_position(self.claw_motor, self.cl_cl)

    def open_claw(self):
        """Pulls Oscar's claw down and opens it"""
        self.BP.set_motor_position(self.claw_motor, self.op_cl)  

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

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.process = Process(target=self.updateOdometry, args=())
        self.process.start()
        print("PID: ", self.process.pid)
        # we don't really need to pass the shared params x, y, th, finished,
        # because they are part of the class, so visible within updateOdometry in any case,
        # but it's just to show an example of process receiving params

    def updateOdometry(self):
        """
        The same as updateOdometry, but less scary (~º3º)~          (OJO QUE AQUI ESTOY HACIENDO COSAS QUE POSBOT ME HACE EN SAGE)
        """
        while not self.finished.value:
            tIni = time.clock()
            v, w = self.readSpeed()     # v is in m/s and w in rad/s
            # implementation of the *formulas*
            if w == 0:
                th = 0
                s = v * self.odometry_period
            else:
                th = w * self.odometry_period
                s = (v / w) * th
            self.lock_odometry.acquire()    # the following operations are non-atomic, so we lock them so the variables cannot be modified during its execution
            self.x.value += s * math.cos((self.th.value + th/2))
            self.y.value += s * math.sin((self.th.value + th/2))
            self.th.value = sage.norm_pi(self.th.value + th)
            self.lock_odometry.acquire()
            tEnd = time.clock()
            time.sleep(self.odometry_period - (tEnd-tIni))    # 2 mimir que es 2 late
            logging.info('Odometry\'s execution time: {}'.format(tEnd-tIni))
            with open(self.odometry_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=' ')
                writer.writerow([self.x.value, self.y.value, self.th.value])
        logging.info("Odometry was stopped... :(")

    # Stop the odometry thread.
    def toaPolla(self):
        #self.BP.set_motor_power(self.left_motor, 90)
        #self.BP.set_motor_power(self.right_motor, 90)
        self.setSpeed(0.3, 0)
    def stopOdometry(self):
        self.finished.value = True

    def stopRobot(self):
        """
        Stops the robot
        """
        self.setSpeed(0, 0)
        #logging.info('Stopped the robot!')  

    def kill(self):
        self.stopRobot()
        self.stopOdometry()
        #logging.warning('The robot has been annihilated')

    def setup(self):
        """
        Sets Oscar ready to fight: sets limits, detects claw position, checks the camera, etc (maybe a lil brake check / spin check would be okay too?)
        """
        #logging.basicConfig(filename=self.log_file, encoding='utf-8', datefmt='%d/%m/%Y %H:%M:%S')
        #logging.info('Started the robot!')
        self.stopRobot()
        self.BP.reset_motor_encoder(self.left_motor + self.right_motor)
        self.BP.set_motor_limits(self.claw_motor, 50, 60)
        self.op_cl = self.BP.get_motor_encoder(self.claw_motor)
        self.cl_cl = self.op_cl - 225


