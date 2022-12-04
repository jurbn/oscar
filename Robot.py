#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division
from distutils.debug import DEBUG

import sage
import brickpi3 # import the BrickPi3 drivers
import movement as mv
import go_to4 as gt

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

        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])

        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        self.finished = Value('b',1)
        self.w_max = 100
        self.v_max = 0.5
        
        self.location = [0, 0, 0]
        self.offset = init_position

        self.map_file = 'maps/mapa3.txt'
        self.cell = [0, 0]
        [self.map_size, self.map] = sage.read_map(self.map)

        self.radius = 0.028
        self.length = 0.15
        self.op_cl = 275
        self.cl_cl = 0

        self.BP = brickpi3.BrickPi3()

        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.claw_motor = self.BP.PORT_C

        self.BP.reset_all()

        self.gyro = self.BP.PORT_2
        self.ultrasonic = self.BP.PORT_1
        self.cam = cv.VideoCapture(0)
        self.reduction = 0.5

        self.BP.set_sensor_type(self.gyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.BP.set_sensor_type(self.ultrasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)

        self.BP.offset_motor_encoder(self.claw_motor, self.BP.get_motor_encoder(self.claw_motor))
        self.BP.set_motor_limits(self.claw_motor, 100, 400)

        self.lock_odometry = Lock()
        self.odometry_period = 0.01
        self.blob_period = 0.5 
        self.last_seen_left = False
        self.changed = False

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

    #################
    # MAP FUNCTIONS #
    #################

    def navigateMap(self, origin, goal):    # TODO: cambiar en odometry que actualice robot.cell y go_to que tenga como parámetro el array del move y no el int
        """The robot navigates the map to reach a given goal"""
        [size, map] = sage.read_map(self.map)
        #origin = sage.them_to_us(size, origin)
        goal = sage.tile2array(size, goal)
        grid = sage.generate_grid(map, goal)
        print(grid)
        finished = False
        moves = [[0,-1], [1,-1], [1,0], [1,1], [0,1], [-1,1], [-1,0], [-1,-1]]
        offset_angle = 0
        while not finished: # cuando no haya acabado, sigue recorriendo el mapa
            #if self.BP.get_sensor(self.ultrasonic) < 20:    # si encuentra un obstaculo, remakea el mapa
            #    self.remakeMap(size, map, goal, origin)
            arr_pos = sage.pos2array(size, map, [self.x.value, self.y.value]) # calcula la pos que tiene en el mapa
            print('im in {}, {}'.format(arr_pos, self.th.value))
            print('MY GRID VALUE IS {}'.format(grid[int(arr_pos[0]), int(arr_pos[1])]))
            if grid[int(arr_pos[0]), int(arr_pos[1])] == 0:  # si el valor del grid de mi pos es 0, he acabado!!!  
                finished = True
            else:   # si no he acabado, valoro que movimiento es el mejor (el que sea un número más bajo al que tengo ahora)
                smallest_value = grid[int(arr_pos[0]), int(arr_pos[1])]     # el valor más pequeño empieza siendo el MIO

                if sage.is_near_angle(self.th.value, math.pi):  # sacamos el offset del movimiento relativo!
                    offset_angle = 0
                elif sage.is_near_angle(self.th.value, math.pi/2):
                    offset_angle = 2
                elif sage.is_near_angle(self.th.value, 0):
                    offset_angle = 4
                elif sage.is_near_angle(self.th.value, -math.pi/2):
                    offset_angle = 6

                [relative_move, abs_destination, clockwise] = sage.next_cell(moves, offset_angle, arr_pos, smallest_value)  # sacamos la siguiente celda a la que tenemos que ir!
                
                arrived = mv.go_to_cell(self, map, relative_move, abs_destination, clockwise)   # recorremos el mapa hasta llegar a la siguiente celda

                if not arrived: # no ha llegao
                    self.setSpeed(0, 0)
                    grid = self.remakeMap(size, map, goal, origin)
                else:
                    sage.draw_map(grid, offset_angle/2, arr_pos)

                    time.sleep(1)

        self.setSpeed(0, 0)
        return True

    def remakeMap(self, size, map, goal, origin):
        """Updates the map when the robot encounters an obstacle"""
        pos = sage.pos2array(size, map, [self.x.value, self.y.value])
        th = self.th.value
        # check which direction is it facing...
        print('looking one way')
        while (abs(th-self.th.value) < math.pi/4):
            self.setSpeed(0, math.pi/4)
        self.setSpeed(0, 0)
        obstacle_right = self.BP.get_sensor(self.ultrasonic) < 100
        self.setSpeed(0, -math.pi/4)
        time.sleep(0.1)
        while (abs(th-self.th.value) < math.pi/4): # MAL MAL MAL MAL MAL
            print('looking the other way')
            self.setSpeed(0, -math.pi/4)
        self.setSpeed(0, 0)
        obstacle_left = self.BP.get_sensor(self.ultrasonic) < 100
        if th <= math.pi/4 and th >= -math.pi/4: # mirando hacia arriba
            map[int(pos[0]), int(pos[1])-1] = 0
            map[int(pos[0])-1, int(pos[1])-1] = 1 * (not obstacle_left)
            map[int(pos[0])+1, int(pos[1])-1] = 1 * (not obstacle_right)
        elif th >= math.pi/4 and th <= 3*math.pi/4: # mirando izda
            map[int(pos[0])-1, int(pos[1])] = 0
            map[int(pos[0])-1, int(pos[1])+1] = 1 * (not obstacle_left)
            map[int(pos[0])-1, int(pos[1])-1] = 1 * (not obstacle_right)
        elif th >= 3*math.pi/4 and th <= -3*math.pi/4: # mirando abajo
            map[int(pos[0]), int(pos[1])+1] = 0
            map[int(pos[0])+1, int(pos[1])+1] = 1 * (not obstacle_left)
            map[int(pos[0])-1, int(pos[1])+1] = 1 * (not obstacle_right) 
        elif th <= -math.pi/4 and th >= -3*math.pi/4: # mirando dcha
            map[int(pos[0])+1, int(pos[1])] = 0
            map[int(pos[0])+1, int(pos[1])-1] = 1 * (not obstacle_left)
            map[int(pos[0])+1, int(pos[1])+1] = 1 * (not obstacle_right) 
        grid = sage.generate_grid(map, goal)
        print(grid)
        time.sleep(0.05)
        return grid

    def startTeabag():
        self.finish_tb.value = False
        self.teabag = Process(target=self.updateTeabag, args=())
        self.teabag.start()
        logging.info('Teabag started, PID: {}'.format(self.teabag.pid))

    def updateTeabag():
        while not self.finish_tb.value:
            tIni = time.clock()
            try:
                distance = self.BP.get_sensor(self.ultrasonic)
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
            print("There was an error while reading the speed of the motors")
        return v, w_rad
    
    def takePic(self, save: str = None):    # seteamos que ser un string con default value None (Null o whatever en java)
        """Takes a nice picture and stores it as the save parameter"""
        ret = False
        while not ret:
            ret, frame = self.cam.read()
        frame  = cv.rotate(frame, cv.ROTATE_180)
        frame = cv.resize(frame, None, fx = self.reduction, fy = self.reduction, interpolation = cv.INTER_LANCZOS4)
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

    def searchBall(self):
        """Uses the camera to locate the ball"""
        found = False
        if  self.last_seen_left:
            w = 1.5
        else: w = -1.5
        while not found:
            #tIni = time.clock()
            frame = self.takePic()
            blob = sage.get_blob(frame = frame)
            if blob:
                found = True
                logging.info('Found the ball! Approaching...')
            else:
                self.setSpeed(0, w)   # gira relativamente rápido para simplemente localizarla (SI TENEMOS LAST_POS, GIRAR HACIA AHI!!!!!!!)
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
                if blob.pt[0] > 160: #derecha
                    self.last_seen_left = False
                elif blob.pt[0] < 160:   #izquierda
                    self.last_seen_left = True
                if blob.size >= 60:   #por ejemplo
                    ready = True
                    logging.info('Close enough to the ball, size is: {}'.format(blob.size))
                else:
                    v = ((72.5 - blob.size) / 72.5) * 0.25
                    w = ((160 - blob.pt[0]) / 160) * (math.pi/6)
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
            if blob:
                logging.info('The ball\'s x position is: {}'.format(blob.pt[0]))
                if blob.pt[0] > 162: #un poco mas de la mitad
                    self.setSpeed(0, -0.1)
                    self.last_seen_left = False
                elif blob.pt[0] < 158:   # un poco menos de la mitad
                    self.setSpeed(0, 0.1)
                    self.last_seen_left = True
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
                    valid = True
                time.sleep(self.odometry_period-time.clock()+tIni)
            distance_array.append(data)
            tEnd = time.clock()
            time.sleep(self.odometry_period-tEnd+tIni)
        distance = np.median(distance_array) / 100 - 0.1# lo dividimos para 100 pq las unidades del sensor son cm Y LE METEMOS OFFSET DE 5CM
        if distance > 0.30:
            return False
        logging.info('The distance to be covered is: {} meters'.format(distance))
        self.BP.set_motor_position(self.claw_motor, self.op_cl)
        time.sleep(0.5)
        #point = sage.absolute_offset(self, distance)
        #while not sage.is_near(self, point, 0.01):
        self.setSpeed(0.1, 0)
        time.sleep(distance/0.1 + 0.12)
        self.setSpeed(0, 0)
        self.BP.set_motor_position(self.claw_motor, self.cl_cl)
        time.sleep(0.6)
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

    def ballCaught(self):
        """Checks wether or not oscar got the ball"""
        frame = self.takePic()[219:239, 139:179]
        rows, cols, _ = frame.shape
        rowsA, colsA = 0,0

        print ('tamaño imagen: {}x{} pixels comprobados: {}x{} (origen en centro inferior: ({}, 0))'.format(rows,cols,rowsA,colsA,cols/2))

        return 'holi soy ballCaught y estoy incompleta'
            
    #####################
    # ODOMETRY THINGIES #
    #####################

    def readOdometry(self): #TODO: borrar esto pq no se usa y ahora tenemos self.location
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
        [enc_l_1, enc_r_1] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]        
        [enc_l_1, enc_r_1] = [0, 0]
        time.sleep(self.odometry_period)
        while not self.finished.value:
            tIni = time.clock()
            #if self.changed:
            #    self.setSpeed(self.v.value, 0)
            th = sage.norm_pi(self.offset[2] - math.radians(self.BP.get_sensor(self.gyro)[0]))
            [enc_l_2, enc_r_2] = [self.BP.get_motor_encoder(self.left_motor), self.BP.get_motor_encoder(self.right_motor)]
            v_l = math.radians((enc_l_2 - enc_l_1) / self.odometry_period) * self.radius
            v_r = math.radians((enc_r_2 - enc_r_1) / self.odometry_period) * self.radius
            [enc_l_1, enc_r_1] = [enc_l_2, enc_r_2]
            
            w = math.radians(self.BP.get_sensor(self.gyro)[1])
            #if (self.w.value == 0) and (w != 0):
            #    self.changed = True
            #    self.setSpeed(self.v.value, w)
            try:
                r = (self.length / 2) * (v_l + v_r) / (v_r - v_l)
                v = r * w
            except Exception:
                v = v_l
            if self.v.value == 0:
                v = 0
            s = v*self.odometry_period

            self.lock_odometry.acquire()
            self.x.value += s * math.cos(th)*2
            self.y.value += s * math.sin(th)*2
            self.th.value = th
            self.lock_odometry.release()
            self.location = [self.x.value, self.y.value, self.th.value]
            self.cell = sage.pos2array(self.map_size, self.map, self.location)  # updates the cell!
            
            with open(self.odometry_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([self.x.value, self.y.value, self.th.value])
            tEnd = time.clock()
            time.sleep(self.odometry_period - tEnd + tIni)
        
    def stopOdometry(self):
        """Must be called when a stop on odometry is desired"""
        logging.info('Odometry was stopped')
        self.finished.value = True
