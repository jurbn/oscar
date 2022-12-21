import logging
from tabulate import tabulate
import numpy as np
import helpers.location
import math

def draw_map(grid, robot, direction = None, pos = None, tile_map = False):
    """
    Prints the grid and, if given, the direction of the robot
    """
    logging.debug('Y \n↑ \no → X')
    arrow_list = ['↑', '→', '↓', '←']
    ascii_grid = grid.astype(int).tolist()
    #sustituimos las paredes por 'bloques' (ASCII 219) 
    for j in range (len(grid[0, :])):
        for i in range (len(grid[:, 0])):
            if tile_map and ((j%2 == 0) or (i%2 == 0)):
                ascii_grid[i][j] = '\0'
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
        if -1 < grid_value < smallest_value:  # smallest value starts as the grid value of the cell   
            right_index_1 = norm_index(real_index - 1, max_move)
            right_index_2 = norm_index(real_index - 2, max_move)
            left_index_1 = norm_index(real_index + 1, max_move)
            left_index_2 = norm_index(real_index + 2, max_move)

            #TODO: que priorice clockwise o no en pl<n eficiencia o algo
            if i % 2 == 0:    # if its a lateral
                #can go straight to the tile:
                if are_cells_connected([arr_pos, possible_cell], grid): 
                    clockwise = 3
                    relative_move = i
                    abs_destination = [possible_cell]
                    smallest_value = grid_value
                #can make a big clockwise turn:
                #TODO: llamar a go_to_cell varias veces en estos casos de arco grande!!!! RUN ARCO ARCO RUN
                elif are_cells_connected([arr_pos, arr_pos+moves[right_index_2], arr_pos+moves[right_index_1], possible_cell], grid): 
                    clockwise = 1   
                    relative_move = i
                    abs_destination = [get_wall(arr_pos, arr_pos+moves[right_index_2]), get_wall(arr_pos+moves[right_index_2], arr_pos+moves[right_index_1]),
                                        get_wall(arr_pos+moves[right_index_1], possible_cell), possible_cell]
                    smallest_value = grid_value
                #can make a big anti-clockwise turn:
                elif are_cells_connected([arr_pos, arr_pos+moves[left_index_2], arr_pos+moves[left_index_1], possible_cell], grid): 
                    clockwise = 0   
                    relative_move = i
                    abs_destination = [get_wall(arr_pos, arr_pos+moves[left_index_2]), get_wall(arr_pos+moves[left_index_2], arr_pos+moves[left_index_1]),
                                        get_wall(arr_pos+moves[left_index_1], possible_cell), possible_cell]
                    smallest_value = grid_value

            else:   # if its a corner 
                #can make a clockwise turn:
                if i == 1:
                    if are_cells_connected([arr_pos, arr_pos+moves[right_index_1], possible_cell], grid):
                        clockwise = 1   
                        relative_move = i
                        abs_destination = [possible_cell]
                        smallest_value = grid_value
                    #can make a anti-clockwise turn:
                    elif are_cells_connected([arr_pos, arr_pos+moves[left_index_1], possible_cell], grid):
                        clockwise = 0   
                        relative_move = i
                        abs_destination = [possible_cell]
                        smallest_value = grid_value
                else:
                    if are_cells_connected([arr_pos, arr_pos+moves[left_index_1], possible_cell], grid):
                        clockwise = 0
                        relative_move = i
                        abs_destination = [possible_cell]
                        smallest_value = grid_value
                    #can make a anti-clockwise turn:
                    elif are_cells_connected([arr_pos, arr_pos+moves[right_index_1], possible_cell], grid):
                        clockwise = 1   
                        relative_move = i
                        abs_destination = [possible_cell]
                        smallest_value = grid_value
    return relative_move, abs_destination, clockwise

def norm_index(i, max_i):
    """Normalizes an index so it doesn't go out of bounds"""
    while i < 0:  
        i += (max_i + 1)
    while i > max_i:
        i -= (max_i + 1)
    return i

def get_rel_index (robot, arr_cell):
    """Gets the relative index of a cell around the robot"""
    robot_cell = helpers.map.pos2array(robot.map_size, [robot.x.value, robot.y.value])
    print('IM IN {}|||{}'.format(robot_cell, robot.x.value, robot.y.value))
    rel_cell = [arr_cell[0] - robot_cell[0], arr_cell[1] - robot_cell[1]]
    rel_cell[0] = helpers.maths.get_sign(rel_cell[0])
    rel_cell[1] = helpers.maths.get_sign(rel_cell[1])
    print('DIFFERENCE IS {}'.format(rel_cell))
    rel_cells = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
    i = 0
    while i < len(rel_cells):
        if (rel_cell == rel_cells[i]):  # or (rel_cell == [cell * 2 for cell in rel_cells[i]]):
            index = i
            break
        else:
            i += 1
    offset = helpers.location.get_robot_quadrant(robot, index=True) * 2
    return norm_index(index - offset, 7)

def are_cells_connected(cells, grid):
    connected = True
    i = 0
    while (i < len(cells) - 1) and connected:
        if(cells[i][0] == cells[i+1][0]): #coinciden en X
            wall_check = [int(cells[i][0]), int((cells[i][1]+cells[i+1][1])/2)]
            connected = (grid[wall_check[0], wall_check[1]] != -1)
        else: #coinciden en Y
            wall_check = [int((cells[i][0] + cells[i+1][0])/2), int(cells[i][1])]
            connected = (grid[wall_check[0], wall_check[1]] != -1)
        i += 1
    return connected

def get_wall(tile1, tile2):
    if(tile1[0] == tile2[0]): #coinciden en X
        wall_pos = [int(tile1[0]), int((tile1[1]+tile2[1])/2)]
    else: #coinciden en Y
        wall_pos = [int((tile1[0] + tile2[0])/2), int(tile1[1])]
    return wall_pos

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

def pos2array_prev(map_size, pos): 
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

def pos2array(map_size, pos):
    """Turns real-world coordinates into their equivalent map positions considering the walls have a width of 0.10m"""
    cell = np.array([0, 0], dtype=int)
    cell[1] = (2 * (pos[0]//map_size[2]) + (abs(0.2 - pos[0] % map_size[2]) > 0.15)) +1
    cell[0] = map_size[1] * 2 - (2 * (pos[1]//map_size[2]) + (abs(0.2 - pos[1] % map_size[2]) > 0.15)) -1
    return cell

def base_map2array(map_size, tile):
    x = 2 * map_size[1] - tile[1]
    y = tile[0]
    return np.array([round(x, 0), round(y, 0)])

def tile2array(size, their_coord):
    y = (2 * their_coord[0]) + 1
    x = 2 * size[1] - 1 - 2 * their_coord[1]
    return np.array([round(x, 0), round(y, 0)])

def pos2tile(size, their_coord):
    pass

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