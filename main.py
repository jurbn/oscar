#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
import movement as mv
import logging

def main():
    try:
        oscar = Robot()
        oscar.setup()
        oscar.setSpeed(0, 0)
        time.sleep(2)
        #for i in range(0,5):
        #oscar.setSpeed(0.5,0) #(i/10,0)
            #time.sleep(0.1)
            #print('heeey')
        #time.sleep(5)
        #mv.softStop(oscar)
        #oscar.setSpeed(0,0)
        print('man parao :(')
        #mv.putivuelta(oscar, 0.2, 0.3, 0.6, 0.2)
        #mv.eight(oscar)
        mv.slalom(oscar)
    except KeyboardInterrupt:
        #oscar.stopOdometry()
        oscar.setSpeed(0, 0)

if __name__ == "__main__":
    main()



