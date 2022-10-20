#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
import movement as mv
import logging

def main(args):
    try:
        oscar = Robot()
        oscar.setup()
        oscar.startOdometry()
        #oscar.setSpeed(10, 0)
        #time.sleep(5)
        #oscar.setSpeed(0, 10)
        #time.sleep(5)
        #oscar.stopOdometry()
        #oscar.stopRobot()      
        #mv.putivuelta(oscar, 0.2, 0.3, 0.6, 0.2)
        oscar.stopRobot()

    except KeyboardInterrupt:
        oscar.stopOdometry()
        oscar.stopRobot()
        logging.info()

if __name__ == "__main__":
    main()



