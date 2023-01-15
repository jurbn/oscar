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
import actions.ball
import actions.dynamic_ball
import helpers.plot
import helpers.map
import traceback

def main(args):
        #########################
        #     initialization    #
        #########################
    try:
        logging.basicConfig(filename='logs/log/' + time.strftime('%y-%m-%d--%H:%M:%S') + '.log', level=logging.DEBUG)
        logging.getLogger().addHandler(logging.StreamHandler())
        logging.info('MAIN: Program started')
        oscar = Robot(init_position=[0.6, 2.6, -math.pi/2]) # init_position=[0.1, 0.9, math.pi] #[0.6, 1.8, -math.pi/2]  [0.2, 1.8, 0]
        logging.info('MAIN: Initial location: {}, {}, {}'.format(oscar.x.value, oscar.y.value, oscar.th.value))
        logging.info('*' + '-'*75 + '*')

        #########################
        #   various functions   #
        #########################
        #sage.plot_animation(oscar)
        if  args.fcn == 'eight':
            mv.eight(oscar)
        elif args.fcn == 'slalom':
            mv.slalom(oscar, 0.4)
        elif args.fcn == 'tronchopocho':
            mv.tronchopocho(oscar)
        elif args.fcn == 'spin':
            mv.spin(oscar, math.pi/2, 0.8, relative = False)
        elif args.fcn == 'run':
            mv.run(oscar, [1, 0])
        elif args.fcn == 'stop':
            oscar.setSpeed(0, 0)
        elif args.fcn == 'ball':
            actions.ball.go_for_ball(oscar, center=False)
        elif args.fcn == 'enc_test':
            mv.enc_test()
        elif args.fcn == 'pictures':
            helpers.vision.show_cam_blobs(oscar)
        elif args.fcn == 'square':
            mv.square(oscar, l = 0.8)
        elif args.fcn == 'ballCheck':
            oscar.ballCaught()
        elif args.fcn == 'reset':
            oscar.BP.reset_all()
        elif args.fcn == 'grid':
            helpers.map.generate_grid(map, goals)
        elif args.fcn == 'arc':
            mv.go_to(oscar, [0.4, 0])
        elif args.fcn == 'nav':
            oscar.updateWithMapFile('res/maps/mapaB_CARRERA2020.txt')
            oscar.forceNewPosition([1.8, 1.4, -math.pi/2])
            oscar.objective = [[1, 4], [4, 4]]
            actions.map.navigate_map(oscar,oscar.objective)
        elif args.fcn == 'dinamo':
            actions.dynamic_ball.go_for_ball(oscar)
        elif args.fcn == 'plot':
            helpers.plot.plot_file('logs/odometry/22-12-21--17:51:57.csv', 'res/maps/mapaA_CARRERA2020.txt')
        elif args.fcn == 'r2d2':
            while True:
                found = helpers.vision.find_my_template(oscar)
                print('I\'ve seen {}'.format('R2D2'*found or 'NOTHING'))
                time.sleep(0.5)
        elif args.fcn == 'whitest':
            black = False
            oscar.setMapByColor(black = black)
            oscar.forceNewPosition([1.4, 1.4, math.pi/2])
            actions.ball.go_for_ball(oscar)
            actions.map.go_to_watchpoint(oscar, black)
            #has_ball = actions.ball.check_caught(oscar, black)
            actions.moves.spin(oscar, math.pi/2, relative = False)
            actions.map.exit_map(oscar, black)

        #########################
        #       the thing       #
        #########################

        #TODO: comprobar con el sensor laser que hay que poner, el color de la baldosa
        # dar valores a las distintas variables in conscecuence to said sensor:
        # map, A o B; slalom, bool+2-1; color de la salida azul o naranja;   

        else:
            black = oscar.isFloorBlack()
            logging.info('MAIN: The floor is {}'.format(black*'BLACK' or 'WHITE'))
            oscar.setMapByColor(black = black)
            actions.moves.run(oscar, [oscar.x.value, oscar.y.value - oscar.map_size[2]-0.10])
            actions.moves.half_eight_short(oscar, black)
            if black:
                oscar.objective = [[1, 4], [4, 4]]
            else: oscar.objective = [[4, 4], [7, 4]]
            actions.map.navigate_map(oscar, oscar.objective, eight_neigh = False)
            #has_ball = False
            #while not has_ball:
            actions.ball.go_for_ball(oscar)
            actions.map.go_to_watchpoint(oscar, black)
            #has_ball = actions.ball.check_caught(oscar, black)
            actions.moves.spin(oscar, math.pi/2, relative = False)
            actions.map.exit_map(oscar, black)

# :) <3

        #########################
        #      closing up       #
        #########################

        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar)

    except KeyboardInterrupt:
        logging.info('*' + '-'*75 + '*')
        logging.warning('KeyboardInterrupt raised')
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar)
    except Exception as error:
        logging.info('*' + '-'*75 + '*')
        logging.warning(traceback.format_exc())
        mv.abrupt_stop(oscar)
        oscar.stopOdometry()
        oscar.BP.reset_all()
        helpers.plot.plot_file(oscar)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f","--fcn",help = "Selec which function you wish to run", type = str)
    args = parser.parse_args()
    main(args)