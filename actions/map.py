import math
import time
import sys
import os
import logging
import traceback

import actions.moves
import helpers.vision
import helpers.maths

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import helpers.location
import helpers.map

def go_to_center(robot):
    if robot.black:
        center = [2*robot.map_size[2], 5*robot.map_size[2]]
    else: center = [5, 5]*robot.map_size[2]
    th = helpers.location.get_angle_between_points([robot.x.value, robot.y.value], center)
    logging.debug('     GO_TO_CENTER: spinning to ABSOLUTE TH {}'.format(th))
    actions.moves.spin(robot, th, relative = False, w = 1)
    logging.debug('     GO_TO_CENTER: running to center: {}'.format(center))
    while not helpers.location.is_near([robot.x.value, robot.y.value], center, threshold=0.07):  
        robot.setSpeed(0.2, 0)

def exit_map(robot, black = True):
    logging.debug('EXIT_MAP: beggining exiting sequence...')
    logging.debug('EXIT_MAP: watchpoint is {}'.format(watchpoint))
    logging.debug('EXIT_MAP: beggining image scan...'.format(watchpoint))
    found = helpers.vision.find_my_template(robot)  # si ha visto a r2d2
    if found:
        logging.info('Ive seen R2D2')
    else:
        logging.info('Ive seen BB8')
    if found and black: # si r2d2 y yo negro, la izda
        exit = [1, 7]
    elif found and not black:   # si r2d2 y yo blanco, izda
        exit = [4, 7]
    elif not found and black:   # si bb8 y yo negro, la dcha
        exit = [4, 7]
    elif not found and not black:   # si bb8 y yo blanco, la dcha
        exit =  [7, 7]
    navigate_map(robot, [], exit, eight_neigh=False)
    front_value = robot.getFrontsonic()
    while not (18 < front_value < 22):
        v = 0.015 * (front_value - 20)
        if v > 0.15: v = 0.15
        elif v < -0.15: v = -0.15
        new_front_value = robot.getFrontsonic()
        robot.setSpeed(v, 0)
        if new_front_value > 60:
            front_value = front_value
        else:
            front_value = new_front_value
    robot.setSpeed(0, 0)
    actions.moves.spin(robot, math.pi/2, relative=False)
    actions.moves.run(robot, [robot.x.value, robot.y.value + robot.map_size[2]])

def go_to_watchpoint (robot, black):
    """The robot faces the watchpoint tile, goes to it and then faces the picture"""
    if black: watchpoint = [3, 6] # se va a la izda a mirar
    else: watchpoint = [5, 6]
    watchpoint_coord = helpers.map.array2pos(robot.map_size, 0, helpers.map.tile2array(robot.map_size, watchpoint))
    watchpoint_coord[0] += 0.1 * helpers.maths.get_sign(4 - watchpoint[0])
    logging.debug('Watchpoint coordinates: {}'.format(watchpoint_coord))
    th = helpers.location.get_angle_between_points([robot.x.value, robot.y.value], watchpoint_coord)
    logging.debug('     GO_TO_WATCHPOINT: spinning to ABSOLUTE TH {}'.format(th))
    actions.moves.spin(robot, th, relative = False, w = 1)
    logging.debug('     GO_TO_WATCHPOINT: running to watchpoint: {}'.format(watchpoint))
    while not helpers.location.is_near([robot.x.value, robot.y.value], watchpoint_coord, threshold=0.1):
    #while (abs(robot.x.value - watchpoint_coord[0]) > 0.04) or (robot.y.value < 2.2) or (robot.y.value > 2.6):  
        robot.setSpeed(0.2, 0)
    #actions.moves.run(robot, watchpoint, correct_trajectory= False, threshold=0.1)
    logging.debug('     GO_TO_WATCHPOINT: facing image (pi/2 (im in {}pi))...'.format(robot.th.value/math.pi))

def navigate_map(robot, origin, goal, eight_neigh = True):    # TODO: cambiar en odometry que actualice robot.cell y go_to que tenga como parámetro el array del move y no el int
    """The robot navigates the map to reach a given goal"""
    [size, map] = helpers.map.read_map(robot.map_file)
    #origin = sage.them_to_us(size, origin)
    try:
        goal = [helpers.map.tile2array(size, goal[0]), helpers.map.tile2array(size, goal[1])]
    except Exception:
        goal = helpers.map.tile2array(size, goal)
    robot.grid = helpers.map.generate_grid(map, goal)
    logging.debug('\n###################### NAVIGATE_MAP ######################\n my turbogoal is: {}(array)'.format(goal))
    finished = False
    moves = [[-2, 0], [-2, +2], [0, 2], [2, 2], [2, 0], [2, -2], [0, -2], [-2, -2]]  # 0,1,2,3,4,5,6,7 en tiles (2arr=1tile)
    #offset_angle = 0   # vamo a no declararlo no vaya a ser que se este saltando los ifs...
    while not finished: # cuando no haya acabado, sigue recorriendo el mapa
        #if robot.BP.get_sensor(robot.ultrasonic) < 20:    # si encuentra un obstaculo, remakea el mapa
        #    robot.remake_map(size, map, goal, origin)
        arr_pos = helpers.map.pos2array(size, [robot.x.value, robot.y.value]) # calcula la pos que tiene en el mapa
        #logging.debug('im in {}, {}'.format(arr_pos, robot.th.value))
        #logging.debug('MY GRID VALUE IS {}'.format(grid[int(arr_pos[0]), int(arr_pos[1])]))
        if robot.grid[int(arr_pos[0]), int(arr_pos[1])] == 0:  # si el valor del grid de mi pos es 0, he acabado!!!  
            logging.info('NAVIGATE_MAP: I\'ve reached my goal')
            finished = True
        else:   # si no he acabado, valoro que movimiento es el mejor (el que sea un número más bajo al que tengo ahora)
            smallest_value = robot.grid[int(arr_pos[0]), int(arr_pos[1])]     # el valor más pequeño empieza siendo el MIO
            #print('Distancia a la pared del frente: {}\n Distancia a la pared de la izq: {}'.format(helpers.map.distance_front_wall(robot, map, size), helpers.map.distance_left_wall(robot, map, size)))
            offset_angle = helpers.location.get_robot_quadrant(robot, index=True) * 2
            helpers.map.draw_map(robot.grid, robot, offset_angle/2, arr_pos, True)
            if (arr_pos[0] % 2 == 0) or (arr_pos[1] % 2 == 0):
                moves = [[move[0]/2, move[1]/2] for move in moves]
            if eight_neigh:
                [relative_move, abs_destinations, clockwise] = helpers.map.next_cell(robot.grid, moves, offset_angle, arr_pos, smallest_value)  # sacamos la siguiente celda a la que tenemos que ir!
            else:
                [relative_move, abs_destinations, clockwise] = helpers.map.next_cell_4(robot.grid, moves, offset_angle, arr_pos, smallest_value)
            for abs_destination in abs_destinations:
                print('-'*20) #TODO: jorge q es esto auxilio
                arrived = go_to_cell(robot, map, relative_move, abs_destination, clockwise, size)
                if not arrived: # no ha llegao
                    logging.debug('GO_TO_CELL: An exception was raised, calling remake_map function (offset angle: {})...\n'.format(offset_angle))
                    offset_angle = helpers.location.get_robot_quadrant(robot, index=True) * 2
                    robot.grid = remake_map(robot, size, map, goal, offset_angle = offset_angle) 
                    break
            logging.debug('\n----------------------------------------------------------------------------\n')
    robot.setSpeed(0, 0)
    return True

def remake_map(robot, size, map, goal, offset_angle = 0):
    """Updates the map when the robot encounters an obstacle"""
    pos = helpers.map.pos2array(size, [robot.x.value, robot.y.value])
    logging.debug('REMAKE_MAP: mi cell is {} and my offset angle is {}\n'.format(pos, offset_angle))
    if offset_angle == 0: # mirando hacia arriba
        map[int(pos[0])-1, int(pos[1])] = 0
        logging.debug('REMAKE_MAP: first case (0)\n')
    elif offset_angle == 6: # mirando izda
        map[int(pos[0]), int(pos[1])-1] = 0
        logging.debug('REMAKE_MAP: second case (6)\n')
    elif offset_angle == 4: # mirando abajo
        map[int(pos[0])+1, int(pos[1])] = 0
        logging.debug('REMAKE_MAP: third case (4)\n')
    elif offset_angle == 2: # mirando dcha
        map[int(pos[0]), int(pos[1])+1] = 0
        logging.debug('REMAKE_MAP: fourth case (2)\n')
    robot.grid = helpers.map.generate_grid(map, goal)
    logging.debug('REMAKE MAP: NEW GRID HAS BEEN GENERATED :D\n')
    return robot.grid

def go_to_cell(robot, map, move, arr_goal, clockwise, map_size): #FIXME: el error tiene que estar aqui next_cell funciona
    """actions.moves the robot given the goal array position being:\n
        actions.moves:          relative goals:\n
        7   0   1       [-1,-1]  [-1,0]  [-1,1]\n
        6   x   2       [0,-1]      x    [0,1]\n
        5   4   3       [1,-1]    [1,0]  [1,1]\n
    with x facing up(0) and y facing left(6)"""
    #logging.DEBUG('GO_TO_CELL: my goal is: {}'.format(arr_goal))
    move = helpers.map.get_rel_index(robot, arr_goal)
    goal = helpers.map.array2pos(map_size, map, arr_goal)
    logging.debug('GO_TO_CELL: relative move: {}'.format(move))
    try:
        if move == 0: 
            logging.debug('GO_TO_CELL: voy recto')
            actions.moves.run(robot, goal, detect_obstacles=True)
        elif move == 1 and (clockwise in (1, 2)):
            logging.debug('GO_TO_CELL: arco a la derecha')
            actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
        elif move == 1 and (clockwise is 0):
            logging.debug('GO_TO_CELL: giro derecha y arco a la izquierda')
            actions.moves.spin(robot, -math.pi/2)
            actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
        elif move == 2:
            logging.debug('GO_TO_CELL: giro derecha y voy recto')
            actions.moves.spin(robot, -math.pi/2)
            actions.moves.run(robot, goal, detect_obstacles=True)
        elif move == 3 and (clockwise in (1, 2)):
            logging.debug('GO_TO_CELL: giro derecha y arco a la derecha')
            actions.moves.spin(robot, -math.pi/2)
            actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
        elif move == 3 and (clockwise is 0):
            logging.debug('GO_TO_CELL: giro 180 y arco a la izquierda')
            actions.moves.spin(robot, math.pi)
            actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
        elif move == 4:
            logging.debug('GO_TO_CELL: giro 180 y voy recto')
            actions.moves.spin(robot, math.pi)
            actions.moves.run(robot, goal, detect_obstacles=True)
        elif move == 5 and (clockwise in (0, 2)):
            logging.debug('GO_TO_CELL: giro izquierda y arco a la izquierda')
            actions.moves.spin(robot, math.pi/2)
            actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
        elif move == 5 and (clockwise is 1):
            logging.debug('GO_TO_CELL: giro 180 y giro derecha')
            actions.moves.spin(robot, math.pi)
            actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
        elif move == 6: 
            logging.debug('GO_TO_CELL: giro izquierda y voy recto')
            actions.moves.spin(robot, math.pi/2)
            actions.moves.run(robot, goal, detect_obstacles=True)
        elif move == 7 and (clockwise in (0, 2)):
            logging.debug('GO_TO_CELL: arco a la izquierda')
            actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
        elif move == 7 and (clockwise is 1):
            logging.debug('GO_TO_CELL: giro izquierda y arco a la derecha')
            actions.moves.spin(robot, math.pi/2)
            actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
        return True
    except Exception:
        print(traceback.format_exc())
        return False