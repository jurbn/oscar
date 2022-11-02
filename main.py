#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
from Robot import Robot
import movement as mv
import logging
import sage

def main(args):
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
        if  args.fcn == 'eight':
            mv.eight(oscar)
        elif args.fcn == 'slalom' :
            mv.slalom(oscar)
        elif args.fcn == 'spin' :
            mv.spin(oscar, 3.1416, 5)
        elif args.fcn == 'run' :
            mv.run(oscar, 0.25, 5)
        elif args.fcn == 'stop' :
            oscar.setSpeed(0, 0)
        elif args.fcn == 'track' :
            oscar.trackObject()
        elif args.fcn == 'enc_test' :
            mv.enc_test()
        else:
            while(True):
                frame = oscar.takePic()
                blob = sage.get_blob(frame=frame)
                if blob:
                    print('Blob position is {}, and its size is: {}'.format(blob.pt, blob.size))
                    time.sleep(0.3)
                else:
                    print('It aint no blobo')
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-f","--fcn",help = "Selec which function you wish to run", type = str)
    args = parser.parse_args()
    main(args)



