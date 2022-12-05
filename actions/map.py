import math
import time
import sys

import moves

sys.path.append('../oscar')
import oscar.helpers as helpers

def navigateMap(robot, origin, goal):    # TODO: cambiar en odometry que actualice robot.cell y go_to que tenga como parámetro el array del move y no el int
    """The robot navigates the map to reach a given goal"""
    [size, map] = helpers.map.read_map(robot.map)
    #origin = sage.them_to_us(size, origin)
    goal = helpers.map.tile2array(size, goal)
    grid = helpers.map.generate_grid(map, goal)
    print(grid)
    finished = False
    moves = [[0,-1], [1,-1], [1,0], [1,1], [0,1], [-1,1], [-1,0], [-1,-1]]
    offset_angle = 0
    while not finished: # cuando no haya acabado, sigue recorriendo el mapa
        #if robot.BP.get_sensor(robot.ultrasonic) < 20:    # si encuentra un obstaculo, remakea el mapa
        #    robot.remakeMap(size, map, goal, origin)
        arr_pos = helpers.map.pos2array(size, map, [robot.x.value, robot.y.value]) # calcula la pos que tiene en el mapa
        print('im in {}, {}'.format(arr_pos, robot.th.value))
        print('MY GRID VALUE IS {}'.format(grid[int(arr_pos[0]), int(arr_pos[1])]))
        if grid[int(arr_pos[0]), int(arr_pos[1])] == 0:  # si el valor del grid de mi pos es 0, he acabado!!!  
            finished = True
        else:   # si no he acabado, valoro que movimiento es el mejor (el que sea un número más bajo al que tengo ahora)
            smallest_value = grid[int(arr_pos[0]), int(arr_pos[1])]     # el valor más pequeño empieza siendo el MIO

            if helpers.location.is_near_angle(robot.th.value, math.pi):  # sacamos el offset del movimiento relativo!
                offset_angle = 0
            elif helpers.location.is_near_angle(robot.th.value, math.pi/2):
                offset_angle = 2
            elif helpers.location.is_near_angle(robot.th.value, 0):
                offset_angle = 4
            elif helpers.location.is_near_angle(robot.th.value, -math.pi/2):
                offset_angle = 6

            [relative_move, abs_destination, clockwise] = helpers.map.next_cell(moves, offset_angle, arr_pos, smallest_value)  # sacamos la siguiente celda a la que tenemos que ir!
            
            arrived = go_to_cell(robot, map, relative_move, abs_destination, clockwise)   # recorremos el mapa hasta llegar a la siguiente celda

            if not arrived: # no ha llegao
                robot.setSpeed(0, 0)
                grid = robot.remakeMap(size, map, goal, origin)
            else:
                helpers.map.draw_map(grid, offset_angle/2, arr_pos)

                time.sleep(1)

    robot.setSpeed(0, 0)
    return True

def remakeMap(robot, size, map, goal, origin):
    """Updates the map when the robot encounters an obstacle"""
    pos = helpers.map.pos2array(size, map, [robot.x.value, robot.y.value])
    th = robot.th.value
    # check which direction is it facing...
    print('looking one way')
    while (abs(th-robot.th.value) < math.pi/4):
        robot.setSpeed(0, math.pi/4)
    robot.setSpeed(0, 0)
    obstacle_right = robot.BP.get_sensor(robot.ultrasonic) < 100
    robot.setSpeed(0, -math.pi/4)
    time.sleep(0.1)
    while (abs(th-robot.th.value) < math.pi/4): # MAL MAL MAL MAL MAL
        print('looking the other way')
        robot.setSpeed(0, -math.pi/4)
    robot.setSpeed(0, 0)
    obstacle_left = robot.BP.get_sensor(robot.ultrasonic) < 100
    if th <= math.pi/4 and th >= -math.pi/4: # mirando hacia arriba
        map[int(pos[0]), int(pos[1])-1] = 0
        map[int(pos[0])-1, int(pos[1])-1] = 1 * (not obstacle_left)
        map[int(pos[0])+1, int(pos[1])-1] = 1 * (not obstacle_right)
    elif th >= math.pi/4 and th <= 3*math.pi/4: # mirando izda
        map[int(pos[0])-1, int(pos[1])] = 0
        map[int(pos[0])-1, int(pos[1])+1] = 1 * (not obstacle_left)
        map[int(pos[0])-1, int(pos[1])-1] = 1 * (not obstacle_right)
    elif th >= 3*math.pi/4 and th <= -3*math.pi/4: # mirando abajo
        map[int(pos[0]), int(pos[1])+1] = 0
        map[int(pos[0])+1, int(pos[1])+1] = 1 * (not obstacle_left)
        map[int(pos[0])-1, int(pos[1])+1] = 1 * (not obstacle_right) 
    elif th <= -math.pi/4 and th >= -3*math.pi/4: # mirando dcha
        map[int(pos[0])+1, int(pos[1])] = 0
        map[int(pos[0])+1, int(pos[1])-1] = 1 * (not obstacle_left)
        map[int(pos[0])+1, int(pos[1])+1] = 1 * (not obstacle_right) 
    grid = helpers.map.generate_grid(map, goal)
    print(grid)
    time.sleep(0.05)
    return grid

def go_to_cell(robot, map, move, goal, clockwise):   #TODO: meter los movimientos y tal
    """moves the robot given the goal array position being:\n
        moves:          relative goals:\n
        7   0   1       [-1,-1]  [0,-1]  [1,-1]\n
        6   x   2       [-1,0]      x    [1,0]\n
        5   4   3       [-1,1]    [0,1]   [1,1]\n
    with x facing up(0) and y facing left(6)"""
    goal = helpers.map(goal)
    if move == 0: 
        print('voy recto')
        moves.run(robot, goal)
    elif (move == 1) and not clockwise:
        print('arco a la derecha')
        moves.arc(robot, goal)
    elif move == 1:
        print('giro derecha y arco a la izquierda')
        moves.spin(robot, -math.pi()/2)
        moves.arc(robot, goal)
    elif move == 2:
        print('giro derecha y voy recto')
        moves.spin(robot, -math.pi()/2)
        moves.run(robot, goal)
    elif (move == 3) and not clockwise:
        print('giro derecha y arco a la derecha')
        moves.spin(robot, -math.pi()/2)
        moves.arc(robot, goal)
    elif move == 3:
        print('giro 180 y arco a la izquierda')
        moves.spin(robot, math.pi())
        moves.arc(robot, goal)
    elif move == 4:
        print('giro 180 y voy recto')
        moves.spin(robot, math.pi())
        moves.run(robot, goal)
    elif (move == 5) and not clockwise:
        print('giro 180 y giro derecha')
        moves.spin(robot, math.pi())
        moves.arc(robot, goal)
    elif move == 5:
        print('giro izquierda y arco a la izquierda')
        moves.spin(robot, math.pi()/2)
        moves.arc(robot, goal)
    elif move == 6: 
        print('giro izquierda y voy recto')
        moves.spin(robot, math.pi()/2)
        moves.run(robot, goal)
    elif (move == 7) and clockwise:
        print('arco a la izquierda')
        moves.arc(robot,goal)
    elif move == 7:
        print('giro izquierda y arco a la derecha')
        moves.spin(robot, math.pi()/2)
        moves.arc(robot,goal)