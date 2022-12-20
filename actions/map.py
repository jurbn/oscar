import math
import time
import sys
import os
import logging
import traceback

import actions.moves

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import helpers.location
import helpers.map

def navigateMap(robot, origin, goal):    # TODO: cambiar en odometry que actualice robot.cell y go_to que tenga como parámetro el array del move y no el int
    """The robot navigates the map to reach a given goal"""
    [size, map] = helpers.map.read_map(robot.map_file)
    #origin = sage.them_to_us(size, origin)
    goal = helpers.map.tile2array(size, goal)
    grid = helpers.map.generate_grid(map, goal)
    finished = False
    moves = [[2, 0], [2, -2], [0, -2], [-2, -2], [-2, 0], [-2, 2], [0, 2], [2, 2]]  # 0,1,2,3,4,5,6,7 en tiles (2arr=1tile)
    #offset_angle = 0   # vamo a no declararlo no vaya a ser que se este saltando los ifs...
    while not finished: # cuando no haya acabado, sigue recorriendo el mapa
        #if robot.BP.get_sensor(robot.ultrasonic) < 20:    # si encuentra un obstaculo, remakea el mapa
        #    robot.remakeMap(size, map, goal, origin)
        arr_pos = helpers.map.pos2array(size, [robot.x.value, robot.y.value]) # calcula la pos que tiene en el mapa
        #logging.debug('im in {}, {}'.format(arr_pos, robot.th.value))
        #logging.debug('MY GRID VALUE IS {}'.format(grid[int(arr_pos[0]), int(arr_pos[1])]))
        if grid[int(arr_pos[0]), int(arr_pos[1])] == 0:  # si el valor del grid de mi pos es 0, he acabado!!!  
            finished = True
        else:   # si no he acabado, valoro que movimiento es el mejor (el que sea un número más bajo al que tengo ahora)
            smallest_value = grid[int(arr_pos[0]), int(arr_pos[1])]     # el valor más pequeño empieza siendo el MIO
            print('Distancia a la pared del frente: {}\n Distancia a la pared de la izq: {}'.format(helpers.map.distance_front_wall(robot, map, size), helpers.map.distance_left_wall(robot, map, size)))
            if helpers.location.is_near_angle(robot.th.value, math.pi/2, threshold=math.pi/5):  # sacamos el offset del movimiento relativo!
                offset_angle = 0
            elif helpers.location.is_near_angle(robot.th.value, 0, threshold=math.pi/5):
                offset_angle = 2
            elif helpers.location.is_near_angle(robot.th.value, -math.pi/2, threshold=math.pi/5):
                offset_angle = 4
            elif helpers.location.is_near_angle(robot.th.value, math.pi, threshold=math.pi/5):
                offset_angle = 6
            helpers.map.draw_map(grid, robot, offset_angle/2, arr_pos)
            [relative_move, abs_destination, clockwise] = helpers.map.next_cell(grid, moves, offset_angle, arr_pos, smallest_value)  # sacamos la siguiente celda a la que tenemos que ir!
            arrived = go_to_cell(robot, map, relative_move, abs_destination, clockwise, size)   # recorremos el mapa hasta llegar a la siguiente celda
            print('ARRIVED???? {}'.format(arrived))
            if not arrived: # no ha llegao
                grid = remakeMap(robot, size, map, goal, offset_angle=offset_angle)
    robot.setSpeed(0, 0)
    return True

def remakeMap(robot, size, map, goal, offset_angle = 0):
    """Updates the map when the robot encounters an obstacle"""
    pos = helpers.map.pos2array(size, [robot.x.value, robot.y.value])
    if offset_angle == 0: # mirando hacia arriba
        map[int(pos[0]), int(pos[1])-1] = 0
    elif offset_angle == 6: # mirando izda
        map[int(pos[0])-1, int(pos[1])] = 0
    elif offset_angle == 4: # mirando abajo
        map[int(pos[0]), int(pos[1])+1] = 0
    elif offset_angle == 2: # mirando dcha
        map[int(pos[0])+1, int(pos[1])] = 0
    grid = helpers.map.generate_grid(map, goal)
    logging.debug('NEW GRID HAS BEEN GENERATED :D')
    return grid

def go_to_cell(robot, map, move, goal, clockwise, map_size):
    """actions.moves the robot given the goal array position being:\n
       actions.moves:     relative goals:\n
        7   0   1       [-2,-2]  [-2,0]  [-2,2]\n
        6   x   2       [0,-2]      x    [0,2]\n
        5   4   3       [2,-2]    [2,0]  [2,2]\n
       with x facing up(0) and y facing left(6)"""
    goal = helpers.map.array2pos(map_size, map, goal)
    logging.debug('RELATIVE CELL: {}, IM GOING TO DO:'.format(move))
    try:
        if move % 2 == 0:   # si arista
            if move == 0:
                if clockwise == 0:  # giro a la derecha y arco izquierda
                    actions.moves.spin(robot, -math.pi/2)
                    actions.moves.run(robot, goal, detect_obstacles=True)
                if clockwise == 1:  # giro izda y arco dcha
                    pass
                elif clockwise == 3:
                    logging.debug('voy recto')
                    actions.moves.run(robot, goal, detect_obstacles=True)
            elif move == 2:
                if clockwise == 0:  # avanza y giro dcha
                    pass
                elif clockwise == 1:# media vuelta, avanza y arco izda
                    pass
                elif clockwise == 3:
                    logging.debug('giro derecha y voy recto')
                    actions.moves.spin(robot, -math.pi/2)
                    actions.moves.run(robot, goal, detect_obstacles=True)
            elif move == 4:
                if clockwise == 0:  # avanza y giro izda
                    pass
                elif clockwise == 1:# media vuelta, avanza y arco dcha
                    pass
                elif clockwise == 3:
                    logging.debug('giro 180 y voy recto')
                    actions.moves.spin(robot, math.pi)
                    actions.moves.run(robot, goal, detect_obstacles=True)
            elif move == 6:
                if clockwise == 0:  # gira izda, avanza y arco izda
                    pass
                elif clockwise == 1:# gira dcha, avanza y arco dcha
                    pass
                elif clockwise == 3: 
                    logging.debug('giro izquierda y voy recto')
                    actions.moves.spin(robot, math.pi/2)
                    actions.moves.run(robot, goal, detect_obstacles=True)
        else:   # si corner
            if move == 1 and (clockwise or clockwise == 2):
                logging.debug('arco a la derecha')
                actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
            elif move == 1 and not clockwise:
                logging.debug('giro derecha y arco a la izquierda')
                actions.moves.spin(robot, -math.pi/2)
                actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
            elif move == 3 and (clockwise or clockwise == 2):
                logging.debug('giro derecha y arco a la derecha')
                actions.moves.spin(robot, -math.pi/2)
                actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
            elif move == 3 and not clockwise:
                logging.debug('giro 180 y arco a la izquierda')
                actions.moves.spin(robot, math.pi)
                actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
            elif move == 5 and (not clockwise or clockwise == 2):
                logging.debug('giro izquierda y arco a la izquierda')
                actions.moves.spin(robot, math.pi/2)
                actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
            elif move == 5 and clockwise:
                logging.debug('giro 180 y giro derecha')
                actions.moves.spin(robot, math.pi)
                actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
            elif move == 7 and (not clockwise or clockwise == 2):
                logging.debug('arco a la izquierda')
                actions.moves.arc(robot, goal, clockwise = False, detect_obstacles=True)
            elif move == 7 and clockwise:
                logging.debug('giro izquierda y arco a la derecha')
                actions.moves.spin(robot, math.pi/2)
                actions.moves.arc(robot, goal, clockwise = True, detect_obstacles=True)
        return True
    except Exception:
        print(traceback.format_exc())
        return False