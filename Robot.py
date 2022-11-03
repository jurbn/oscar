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
import cv2
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
        self.gyro = self.BP.PORT_2

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

        self.BP.set_motor_dps(self.left_motor, math.degrees(rps_left))  # BP works on degrees, so we have to transform it :/
        self.BP.set_motor_dps(self.right_motor, math.degrees(rps_right))

    def readSpeed(self):
        """ Returns Oscar's linear (m/s) and angular (rad/s) speed """
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
        return v, w

    def closeClaw(self):
        """Closes Oscar's claw and pulls it up"""
        self.BP.set_motor_position(self.claw_motor, self.cl_cl)

    def openClaw(self):
        """Pulls Oscar's claw down and opens it"""
        self.BP.set_motor_position(self.claw_motor, self.op_cl)

    def takePic(self, save: str = None):    # seteamos que ser un string con default value None (Null o whatever en java)
        """Takes a nice picture and stores it as the save parameter"""
        vid = cv2.VideoCapture(0)
        ret, frame = vid.read()
        if save:
            cv2.imwrite(save, frame)
        return frame
    
    def searchBall(self, last_pos = None):
        """Uses the camera to locate the ball"""
        found = False
        self.setSpeed(0, 0.5)   # gira relativamente rápido para simplemente localizarla (SI TENEMOS LAST_POS, GIRAR HACIA AHI!!!!!!!)
        while not found:
            time.sleep(0.2) # 5 fotos por segundo como máximo a mi me parece guapo la verdad
            frame = self.takePic(self)
            blob = sage.get_blob(frame = frame)
            if blob:
                found = True
        return blob.pt
    
    def approachBall(self, last_pos = None):
        """The robot will try to get close enough to get the ball"""
        ready = False
        w = 0.1 # VELOCIDADES PROVISIONALES AAAAAAAA
        v = 0.1
        while not ready:
            time.sleep(0.2) # 5 fotos por segundo como máximo a mi me parece guapo la verdad
            frame = self.takePic(self)
            blob = sage.get_blob(frame = frame)
            if blob:
                last_pos = blob.pt
            else:
                self.searchBall(last_pos)
                # POR TERMINAR

                
    def trackObject(self, colorRangeMin=[0, 0, 0], colorRangeMax=[255, 255, 255], targetSize='??', targetShape='??', catch='??'):
        """Locates, tracks and follows any kind of blob by its color, shape, size and, if specified on boolean parameter catch, catches it"""
        finished = False
        targetFound = False
        targetPosition = False
        targetCatched = False
        ##################################

        #ESC = 27
        #KNN = 1
        #MOG2 = 2

        #cam = picamera.PiCamera()

        #cam.resolution = (320, 240)
        #cam.resolution = (640, 480)
        #cam.framerate = 32
        #rawCapture = PiRGBArray(cam, size=(320, 240))
        #rawCapture = PiRGBArray(cam, size=(640, 480))
 
        # allow the camera to warmup
        #time.sleep(0.1)
        
        #for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #    frame = img.array
        #    cv2.imshow('Captura', frame)
        #    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #    # clear the stream in preparation for the next frame
        #    rawCapture.truncate(0)
        #    k = cv2.waitKey(1) & 0xff
        #    if k == ESC:
        #        cam.close()
        #    break
        #cv2.destroyAllWindows()
        
        ##################################################

        while not finished:
            #search for the thingy:
            #mv.spin(w=0.5)
            vid = cv2.VideoCapture(0)
            while not targetFound:
                logging.debug('im lookin for sum')
                ret, frame = vid.read()
                cv2.imwrite('im.png',frame)
                # if (blob?in (im.png)):
                #   targetFound=True

            #vid.release()
            #cv2.destroyAllWindows() #AAAAAAAAAAAAAAAAAAAAaaaaaa
            
            while targetFound: #yay u did it! now go get it!!
                logging.debug('im gon get the thing')
                ret, frame = vid.read()
                cv2.imwrite('im.png', frame)
                # aquí hay que sacar la diferencia entre el área del blob y el deseado
                # v = dA en plan separar en tramos para darle mas velocidad o menos
                # IGUAL ES MEJOR DARLE PRIMERO SOLO W Y LUEGO YA AVANZAR CON V PERO SI

                # sacamos la distancia del centro del blob al centro de la cámara y
                # ajustamos la w en función de eso.
                # lo ideal seria, centro_cam=0, posiciones L <0 y posR>0 para hacer menos cálculos
                
                # le asignamos estos valores al robote:
                self.setSpeed(v,w)
                #if posTarget==posDesired:
                    #self.setSpeed(0,0)
                    #targetPosition = True
                
                if catch:
                    while not targetPosition: #go¡ go to grab¡¡¡
                        pass #inicia la secuencia de agarracion
                else: #corre detras sin mas nose va bien para probar el seguimiento
                    pass

        
    
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
        #[enc_l_1, enc_r_1] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]        
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            tIni = time.clock()
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]

            w = math.radians(self.BP.get_sensor(self.gyro))
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
        self.BP.set_sensor_type(self.gyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.BP.reset_all()
        self.BP.set_motor_limits(self.claw_motor, 50, 60)
        self.op_cl = self.BP.get_motor_encoder(self.claw_motor)
        self.cl_cl = self.op_cl - 225
        self.startOdometry()


