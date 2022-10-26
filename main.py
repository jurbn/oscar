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
        logging.getLogger().addHandler(logging.StreamHandler())
        logging.info('Program started')
        oscar = Robot()
        oscar.setup()
        logging.info('Initial location: {}, {}, {}'.format(oscar.x.value, oscar.y.value, oscar.th.value))
        #sage.plot_animation(oscar)
        #mv.eight(oscar)
        mv.spin(oscar, 3.1416, 5)
        oscar.stopOdometry()
        sage.plot_file(oscar.odometry_file)
        
    except KeyboardInterrupt:
        logging.warning('A keyboard interruption was detected!')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        sage.plot_file(oscar.odometry_file)

if __name__ == "__main__":
    main()



