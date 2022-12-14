import logging
from tabulate import tabulate
import numpy as np
import helpers.location
import math


def draw_map(grid, robot, direction = None, pos = None):
    """
    Prints the grid and, if given, the direction of the robot
    """
    logging.debug('Y \n↑ \no → X')
    arrow_list = ['↑', '→', '↓', '←']
    ascii_grid = grid.astype(int).tolist()
    #sustituimos las paredes por 'bloques' (ASCII 219) 
    for j in range (len(grid[0, :])):
        for i in range (len(grid[:, 0])):
            if (grid[i,j] == -1):
                ascii_grid[i][j] = '█'
    if (direction is not None) and (pos is not None):
        ascii_grid[int(pos[0])][int(pos[1])] = arrow_list[int(direction)]
    logging.debug(tabulate(ascii_grid, tablefmt='grid'))


def read_map(file):
    """This function returns the size and arrays of the given map.
    Size is given by a nxn map of tiles of a mm.
    The map represents the accessible points (0's beign accessible and 1's inaccessible)"""
    size = np.loadtxt(file, max_rows=1)
    size[2] = size[2]/1000
    map = np.loadtxt(file, dtype='int', skiprows=1)
    return size, map

def next_cell(grid, moves, offset_angle, arr_pos, smallest_value):  #FIXME: SMALLEST VALUE CUANDO TIENES QUE REMAKEAR EL MAPA???
    smallest_value = 100
    max_move = len(moves)-1
    for i in range(0, max_move+1):    # i is the relative move
        real_index = offset_angle + i   # real_index is used to get the nearest cells
        while real_index > max_move:    # if we're out of bounds, return to the boundaries
            real_index -= (max_move + 1)    # if its 9, 9-8 = 1 :)
        possible_cell = arr_pos + moves[real_index]
        grid_value = grid[int(possible_cell[0]), int(possible_cell[1])]
        #logging.debug('MY OFFSET VALUE IS {}'.format(offset_angle))
        #logging.debug('im considering {} ({}, {}) w/ grid value {}'.format(i, real_index, possible_cell, grid_value))
        if -1 < grid_value < smallest_value:  # smallest value starts as the grid value of the cell
            if i % 2 == 0:    # if its an edge
                clockwise = 2   # 2 means idgaf
                relative_move = i
                abs_destination = possible_cell
                smallest_value = grid_value
            else:   # if its a corner
                watchout_index_1 = real_index - 1
                watchout_index_2 = real_index + 1
                #logging.debug('indexes are {}, {}'.format(watchout_index_1, watchout_index_2))
                while watchout_index_1 < 0:  # correct the indexes in case we went oob
                    watchout_index_1 += (max_move + 1)
                while watchout_index_2 > max_move:
                    watchout_index_2 -= (max_move + 1)
                #logging.debug('NEW indexes are {}, {}'.format(watchout_index_1, watchout_index_2))
                watchout_1 = arr_pos + moves[watchout_index_1]  # we determine the pos of each watchout on the array
                watchout_2 = arr_pos + moves[watchout_index_2]
                watchout_grid_1 = grid[int(watchout_1[0]), int(watchout_1[1])]    # we get the grid value of each watchout
                watchout_grid_2 = grid[int(watchout_2[0]), int(watchout_2[1])]
                if not (watchout_grid_1 == -1 and watchout_grid_2 == -1):   # if not both of them are -1
                    if watchout_grid_1 == -1:   # if it's just the first
                        clockwise = 0
                    elif watchout_grid_2 == -1:   # if it's just the second
                        clockwise = 1
                    elif watchout_grid_1 != -1 and watchout_grid_2 != -1:
                        clockwise = 2
                relative_move = i
                abs_destination = possible_cell
                smallest_value = grid_value
    return relative_move, abs_destination, clockwise


def next_cell_va_mal(grid, moves, offset_angle, arr_pos, smallest_value):  # TODO: limpiar este codigo que esta guarro guarro
    # FUCIONAMIENTO DE ESTA COSA: basicamente recorremos los moves
    for i in range(len(moves)):
        real_index = i+offset_angle  # the index corresponding to the non-relative values
        # if we surpass the array's limits, loop through the begginning
        while real_index > (len(moves)-1):
            real_index -= len(moves)
        possible_cell = arr_pos + moves[real_index]
        if -1 < grid[int(possible_cell[0]), int(possible_cell[1])] < smallest_value:
            if real_index % 2 != 0:  # if we have to go to a corner, we check its availability
                watchout_1 = real_index - 1  # indexes of moves we have be careful with!
                watchout_2 = real_index + 1
                while watchout_1 < 0:  # if we surpassed the array's limits, loop through the begginning
                    watchout_1 += len(moves)
                # if we surpassed the array's limits, loop through the begginning
                while watchout_2 > (len(moves)-1):
                    watchout_2 -= len(moves)
                watchout_cell_1 = possible_cell + watchout_1
                # PARSEAMOS A INT PQ ESTA HECHO DE FLOATS
                watchout_cell_1 = [int(param) for param in watchout_cell_1]
                watchout_cell_2 = possible_cell + watchout_2
                # PARSEAMOS A INT PQ ESTA HECHO DE FLOATS
                watchout_cell_2 = [int(param) for param in watchout_cell_2]
                # SI NO ESTAN OCUPADAS LAS DOS
                if not ((grid[watchout_cell_1[0], watchout_cell_1[1]] == -1) and (grid[watchout_cell_2[0], watchout_cell_2[1]] == -1)):
                    # si ninguna de las dos esta ocupada!
                    if (grid[watchout_cell_1[0], watchout_cell_1[1]] != -1) and (grid[watchout_cell_2[0], watchout_cell_2[1]] != -1):
                        logging.debug('Ninguna de las celdas está ocupada')
                        clockwise = 2   # clockwise 2 implica que se elige el mejor en cada caso!
                    # si esta ocupada la anterior
                    elif grid[watchout_cell_1[0], watchout_cell_1[1]] == -1:
                        logging.debug('Está ocupada la de la izda')
                        clockwise = 0
                    # si está ocupada la siguiente
                    elif grid[watchout_cell_2[0], watchout_cell_2[1]] == -1:
                        logging.debug('Está ocupada la de la dcha')
                        clockwise = 1
                    relative_move = i
                    abs_destination = possible_cell
                    smallest_value = grid[int(possible_cell[0]), int(possible_cell[1])]
            else:
                relative_move = i
                abs_destination = possible_cell
                smallest_value = grid[int(possible_cell[0]), int(possible_cell[1])]
                clockwise = 2   # clockwise 2 implica que se elige el mejor en cada caso!
            logging.debug('Estoy considerando celda {} con grid_value {}'.format(relative_move, smallest_value))
    return relative_move, abs_destination, clockwise


def generate_grid(map, goal):
    """Generates a grid with the given map and goal.\n
    This is meant to be executed once and use this information to navigate through the circuit (though we may need to run it when we find unexpected obstacles)"""
    grid = -2 * np.ones(map.shape)   # unvisited cells contain -2
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i, j] == 0:
                grid[i, j] = -1     # we set the obstacles and walls to -1
    grid[int(goal[0]), int(goal[1])] = 0      # we set the goal to 0
    # cells that are on the wavefront
    current_cells = np.array([[goal[0], goal[1]]])
    moves = np.array([[+1, 0], [-1, 0], [0, +1], [0, -1]])  # 4 direction neighbours
    finished = False
    while not finished:
        # for n in range(current_cells.size):     # for every cell on the wavefront
        if current_cells.size > 0:
            cell = current_cells[0]     # check if there are any
            for move in moves:  # we create a cell for each move
                next_cell = cell + move
                # we check if they are valid
                if next_cell[0] >= 0 and next_cell[0] < map.shape[0] and next_cell[1] >= 0 and next_cell[1] < map.shape[1]:
                    # we only modify its value if the cell hasn't been explored yet (-2)
                    if grid[int(next_cell[0]), int(next_cell[1])] == -2:
                        # we add it to the wavefront
                        current_cells = np.append(
                            current_cells, [[next_cell[0], next_cell[1]]], axis=0)
                        grid[int(next_cell[0]), int(next_cell[1])] = grid[int(cell[0]), int(cell[1])] + 1   # new cell's value is its parent's +1
            # we delete the current cell from the wavefront
            current_cells = np.delete(current_cells, 0, axis=0)
        else:
            finished = True
    return grid


def array2pos(map_size, map, cell):
    """Turns the map's coordinates into their real-world  positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the middle of every tile or tile border"""
    pos = np.array([0, 0], dtype=float)
    pos[0] = cell[1] * (map_size[2]/2)
    pos[1] = (map_size[1]*2-cell[0]) * (map_size[2]/2)
    return pos

def pos2array(map_size, pos): 
    """Turns real-world coordinates into their equivalent map positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the tiles border position on the map only if it matches exactly that position, otherwise it will return the tile position"""
    cell = np.array([0, 0], dtype=float)
    #cell[0] = (pos[0] - map_size[2]/2) / (map_size[2])
    #cell[1] = (pos[1] - map_size[2]/2) / (map_size[2])
    cell[0] = (pos[0]*2)/map_size[2]
    cell[1] = (pos[1]*2)/map_size[2]
    cell = base_map2array(map_size, cell)
    return cell  # we could also return a modified map maybe with the -3 value in the given position? or the value of a map in that position

def base_map2array(map_size, tile):
    x = 2 * map_size[1] - tile[1]
    y = tile[0]
    return np.array([round(x, 0), round(y, 0)])

def tile2array(size, their_coord):
    y = (2 * their_coord[0]) + 1
    x = 2 * size[1] - 1 - 2 * their_coord[1]
    return np.array([round(x, 0), round(y, 0)])

def distance_front_wall(robot, map, map_size, cells = 3):
    rob_cell = pos2array(map_size, [robot.x.value, robot.y.value])
    rob_cell = [int(cell) for cell in rob_cell]
    if helpers.location.is_near_angle(robot.th.value, math.pi/2, threshold=math.pi/5):  # mira hacia arriba
            for i in range (1, cells):
                if (map[rob_cell[0] + i, rob_cell[1]]) == -1:
                    return 0.2*i 
    elif helpers.location.is_near_angle(robot.th.value, 0, threshold=math.pi/5): #derecha
        for i in range (1, cells):
                if (map[rob_cell[0], rob_cell[1] + i]) == -1:
                    return 0.2*i 
    elif helpers.location.is_near_angle(robot.th.value, -math.pi/2, threshold=math.pi/5): #abajo
        for i in range (1, cells):
                if (map[rob_cell[0] - i, rob_cell[1]]) == -1:
                    return 0.2*i
    elif helpers.location.is_near_angle(robot.th.value, math.pi, threshold=math.pi/5): #izquierda
        for i in range (1, cells):
                if (map[rob_cell[0], rob_cell[1] - i]) == -1:
                    return 0.2*i
    return None

def distance_left_wall(robot, map, map_size, cells = 3):
    rob_cell = pos2array(map_size, [robot.x.value, robot.y.value])
    rob_cell = [int(cell) for cell in rob_cell]
    if helpers.location.is_near_angle(robot.th.value, math.pi/2, threshold=math.pi/5):  # mira hacia arriba
            for i in range (1, cells):
                if (map[rob_cell[0], rob_cell[1] - i]) == -1:
                    return 0.2*i 
    elif helpers.location.is_near_angle(robot.th.value, 0, threshold=math.pi/5): #derecha
        for i in range (1, cells):
                if (map[rob_cell[0] + i, rob_cell[1]]) == -1:
                    return 0.2*i 
    elif helpers.location.is_near_angle(robot.th.value, -math.pi/2, threshold=math.pi/5): #abajo
        for i in range (1, cells):
                if (map[rob_cell[0], rob_cell[1] +i]) == -1:
                    return 0.2*i
    elif helpers.location.is_near_angle(robot.th.value, math.pi, threshold=math.pi/5): #izquierda
        for i in range (1, cells):
                if (map[rob_cell[0] - i, rob_cell[1]]) == -1:
                    return 0.2*i
    return None
