#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import math

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.radius = 0
        self.length = 0

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B

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

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period
        self.P = 1.0



    def setSpeed(self, v, w):
        """ Sets the speed of both motors to achieve the given speed parameters (angular speed must be in rps) """
        print("setting speed to %.2f m/s and %.2f rad/s" % (v, w))

        rps_left = (v - (w * self.length) / 2) / self.radius
        rps_right = (v + (w* self.length) / 2) / self.radius

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        self.BP.set_motor_dps(self.left_motor, math.degrees(rps_left))  # BP works on degrees, so we have to adapt it :/
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

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=(self.x, self.y, self.th, self.finished))
        self.p.start()
        print("PID: ", self.p.pid)
        # we don't really need to pass the shared params x, y, th, finished,
        # because they are part of the class, so visible within updateOdometry in any case,
        # but it's just to show an example of process receiving params

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self, x_odo, y_odo, th_odo, finished):
        """ To be filled ...  """

        while not finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                Y=  %d, th=  %d \n" %(x_odo.value, y_odo.value, th_odo.value) )
            #print("Dummy update of odometry ...., X=  %.2f" %(x_odo.value) )

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            # Operations like += which involve a read and write are not atomic.
            with x_odo.get_lock():
                x_odo.value+=1

            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            #x_odo.value+=1
            y_odo.value+=1
            th_odo.value+=1
            self.lock_odometry.release()

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                #print("Reading encoder values ....")
                sys.stdout.write("Reading encoder values .... \n")
                #[encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(x_odo.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(x_odo.value, y_odo.value, th_odo.value))

    def updateOdometry2(self):
        """
        The same as updateOdometry, but less scary (~º3º)~
        """
        while not self.finished.value:
            tIni = time.clock()
            v, w = self.readSpeed()     # v is in m/s and w in rad/s
            # implementation of the *formulas*
            if w == 0:
                th = 0
                s = v * self.P
            else:
                th = w * self.P
                s = (v / w) * th
            self.lock_odometry.acquire()    # the following operations will be performed at the same time (or somethin?)
            self.x += s * math.cos((self.th + th/2))
            self.y += s * math.sin((self.th + th/2))
            self.th += th
            self.lock_odometry.acquire()
            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))    # 2 mimir que es too late
        print("Odometry was stopped... :(")
            



    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()

