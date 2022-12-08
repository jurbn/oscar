#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
import logging
import math
import numpy as np
from Robot import Robot
import actions.moves as mv
import actions.map
import helpers.plot
import traceback

def main(args):

        #########################
        #     initialization    #
        #########################
    try:
        logging.basicConfig(filename='logs/log/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.log', level=logging.DEBUG)
        logging.getLogger().addHandler(logging.StreamHandler())
        logging.info('Program started')
        oscar = Robot(init_position=[0.2, 1.8, math.pi])
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
            mv.run(oscar, 0.1, 10)
        elif args.fcn == 'stop' :
            oscar.setSpeed(0, 0)
        elif args.fcn == 'ball' :
            oscar.goForBall()
        elif args.fcn == 'enc_test' :
            mv.enc_test()
        elif args.fcn == 'pictures':
            helpers.vision.show_cam_blobs(oscar) 
        elif args.fcn == 'square':
            mv.square(oscar, l = 0.4)
        elif args.fcn == 'ballCheck':
            oscar.ballCaught()
        elif args.fcn == 'reset':
            oscar.BP.reset_all()
        elif args.fcn == 'arc':
            mv.go_to(oscar, [0.4, 0])
        elif args.fcn == 'nav':
            actions.map.navigateMap(oscar, [0, 4], [4, 1])
        
        #########################
        #       the thing       #
        #########################

        #TODO: comprobar con el sensor laser que hay que poner, el color de la baldosa
        # dar valores a las distintas variables in conscecuence to said sensor:
        # map, A o B; slalom, bool+2-1; color de la salida azul o naranja;
        # 
        #
        #
        #

        #########################
        #      closing up       #
        #########################

        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar.odometry_file)

    except KeyboardInterrupt:
        logging.warning('KeyboardInterrupt raised')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar.odometry_file)
    except Exception as error:
        logging.warning(traceback.format_exc())
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar.odometry_file)
   
   
    # In the event of a keyboard interruption, we stop the robot's movement 
    # and close the odometry process, as well as log the event as a warning.    
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f","--fcn",help = "Selec which function you wish to run", type = str)
    args = parser.parse_args()
    main(args)


