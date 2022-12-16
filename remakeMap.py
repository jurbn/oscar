
import helpers.map
import actions.moves
import time
import logging
import math

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