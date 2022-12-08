import logging
from tabulate import tabulate
import numpy as np


def draw_map(grid, robot, direction = None, pos = None):
    """
    Prints the grid and, if given, the direction of the robot
    """
    logging.debug("""Y \n↑ \no → X      MY POSITION IS [{}, {}, {}]""".format(robot.x.value, robot.y.value, robot.th.value))
    lim_str = '+---' * len(grid[0, :]) + '+'
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


def next_cell(grid, moves, offset_angle, arr_pos, smallest_value):  # TODO: limpiar este codigo que esta guarro guarro
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
                        clockwise = False   # TODO: pongo esto por ejemplo, pero podría elegir qué opción es mejor!
                    # si esta ocupada la anterior
                    elif grid[watchout_cell_1[0], watchout_cell_1[1]] == -1:
                        clockwise = False
                    # si está ocupada la siguiente
                    elif grid[watchout_cell_2[0], watchout_cell_2[1]] == -1:
                        clockwise = True
                    relative_move = i
                    abs_destination = possible_cell
                    smallest_value = grid[int(possible_cell[0]), int(possible_cell[1])]
            else:
                relative_move = i
                abs_destination = possible_cell
                smallest_value = grid[int(possible_cell[0]), int(possible_cell[1])]
                clockwise = False
    return relative_move, abs_destination, clockwise
        

def generate_grid(map, goal):
    """Generates a grid with the given map and goal.\n
    This is meant to be executed once and use this information to navigate through the circuit (though we may need to run it when we find unexpected obstacles)"""
    grid = -2 * np.ones(map.shape)   # unvisited cells contain -2
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i, j] == 0:
                grid[i, j] = -1     # we set the obstacles and walls to -1
    grid[int(goal[1]), int(goal[0])] = 0      # we set the goal to 0
    # cells that are on the wavefront
    current_cells = np.array([[goal[1], goal[0]]])
    moves = np.array([[+1, 0], [-1, 0], [0, +1], [0, -1]]
                     )  # 4 direction neighbours
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
                        grid[int(next_cell[0]), int(next_cell[1])] = grid[int(cell[0]), int(
                            cell[1])] + 1   # new cell's value is its parent's +1
            # we delete the current cell from the wavefront
            current_cells = np.delete(current_cells, 0, axis=0)
        else:
            finished = True
    return grid


def pos2array(map_size, map, pos):
    """Turns real-world coordinates into their equivalent map positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the tiles border position on the map only if it matches exactly that position, otherwise it will return the tile position"""
    cell = np.array([0, 0])
    #cell[0] = (pos[0] - map_size[2]/2) / (map_size[2])
    #cell[1] = (pos[1] - map_size[2]/2) / (map_size[2])
    cell[0] = (pos[0]*2)/map_size[2]
    cell[1] = (pos[1]*2)/map_size[2]
    cell = tile2array(map_size, cell)
    cell = cell.astype(np.float32)
    return cell  # we could also return a modified map maybe with the -3 value in the given position? or the value of a map in that position


def array2pos(map_size, map, cell):
    """Turns the map's coordinates into their real-world  positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the middle of every tile or tile border"""
    logging.debug(map_size)
    logging.debug(cell)
    pos = np.array([0, 0], dtype=np.float32)
    pos[0] = cell[0]/2 * map_size[2]
    pos[1] = (map_size[1]-(cell[1])/2)*map_size[2]
    return pos


def tile2array(map_size, tile):
    #x = (2 * their_coord[0]) + 1
    #y = 2 * size[1] - 1 - 2 * their_coord[1]
    x = 2*map_size[1] - tile[1]
    y = tile[0]
    return np.array([x, y])
