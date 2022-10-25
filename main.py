#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
from Robot import Robot
import movement as mv
import logging
import sage

def main():
    try:
        logging.basicConfig(filename='log/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.log', level=logging.DEBUG)
        logging.info('Program started')
        oscar = Robot()
        oscar.setup()
        mv.eight(oscar)
        sage.plot_file(oscar.odometry_file)
    except KeyboardInterrupt:
        logging.warning('A keyboard interruption was detected!')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        sage.plot_file(oscar.odometry_file)

if __name__ == "__main__":
    main()



