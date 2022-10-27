#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
from Robot import Robot
import movement as mv
import logging
import sage

def main(function=''):
    try:        
        #########################
        #     initialization    #
        #########################

        logging.basicConfig(filename='log/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.log', level=logging.DEBUG)
        logging.getLogger().addHandler(logging.StreamHandler())
        logging.info('Program started')
        oscar = Robot()
        oscar.setup()
        logging.info('Initial location: {}, {}, {}'.format(oscar.x.value, oscar.y.value, oscar.th.value))
        
        #########################
        #   various functions   #
        #########################

        #sage.plot_animation(oscar)
        if function == 'eight':
            mv.eight(oscar)
        elif function == 'slalom' :
            mv.slalom(oscar)
        elif function == 'spin' :
            mv.spin(oscar, 3.1416, 5)
        elif function == 'run' :
            mv.run(oscar, 0.25, 5)
        elif function == 'stop' :
            oscar.setSpeed(0, 0)
        elif function == 'track' :
            oscar.trackObject()
        
        #########################
        #      closing up       #
        #########################

        oscar.stopOdometry()
        sage.plot_file(oscar.odometry_file)

    
    # In the event of a keyboard interruption, we stop the robot's movement 
    # and close the odometry process, as well as log the event as a warning.    
    except KeyboardInterrupt:
        logging.warning('A keyboard interruption was detected!')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        sage.plot_file(oscar.odometry_file)

if __name__ == "__main__":
    main()



