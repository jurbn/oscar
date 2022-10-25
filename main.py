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
        logging.basicConfig(filename='log/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.log', level=logging.DEBUG)
        logging.info('Program started')
        oscar = Robot()
        oscar.setup()
        oscar.setSpeed(0, 0)
        time.sleep(2)
       
        print('man parao :(')
        mv.slalom(oscar)
    except KeyboardInterrupt:
        logging.warning('A keyboard interruption was detected!')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()

if __name__ == "__main__":
    main()



