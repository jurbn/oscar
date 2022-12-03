# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division       #

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as FuncAnimation
import time
import pandas as pd
import cv2 as cv
import logging
from datetime import datetime
from tabulate import tabulate


params = cv.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 50
params.maxArea = 1000000
params.filterByCircularity = True
params.minCircularity = 0.1
params.maxCircularity = 1
params.filterByColor = False
params.filterByConvexity = False
detector = cv.SimpleBlobDetector_create(params)
r11 = np.array([0, 70, 50])
r12 = np.array([10, 255, 255])
r21 = np.array([170, 70, 50])
r22 = np.array([180, 255, 255])

##################
# PLOT FUNCTIONS #
##################


def plot_file(file_name):
    df = pd.read_csv(file_name)
    plt.axis('equal')
    plt.plot(df['x'], df['y'])
    plt.show()


def plot_animation(robot):
    fig = plt.figure(figsize=(6, 3))
    x = [0]
    y = [0]
    ln, = plt.plot(x, y, '-')

    def update(frame):
        x.append(robot.x.value)
        y.append(robot.y.value)
        ln.set_data(x, y)
        fig.gca().relim()
        fig.gca().autoscale_view()
        return ln,
    animation = FuncAnimation(fig, update_plot, interval=50)
    plt.show()


def pos_bot(vw, x_w_r, t):
    """
    Returns the position of the robot with vc=[v, w] during t seconds and starting on x_w_r.
    """
    if vw[1] == 0:   # w=0
        x_r_r2 = np.array([vw[0]*t, 0, 0])
    else:
        r = vw[0] / vw[1]
        th = vw[1] * t
        al = (np.pi - th) / 2
        x_r_r2 = np.array([r*chord(th)*np.sin(al), r*chord(th)*np.cos(al), th])
    x_w_r2 = loc(np.matmul(hom(x_w_r), hom(x_r_r2)))   # new location x_w_r
    return x_w_r2

##################
# MATH FUNCTIONS #
##################


def hom(x: np.array):
    """
    Returns the Transformation matrix based on the location vector
    """
    return np.matrix([[np.cos(x[2]), -np.sin(x[2]), x[0]], [np.sin(x[2]), np.cos(x[2]), x[1]], [0, 0, 1]])


def loc(T):
    """
    Returns the location vector based on the transformation matrix
    """
    return np.array([T.item(0, 2), T.item(1, 2), np.arctan2(T.item(1, 0), T.item(0, 0))])


def chord(th):
    """
    Returns the cord function of the given angle
    """
    return 2*np.sin(th/2)


def norm_pi(th):
    """
    Normalizes the given angle (forces it to be between pi and -pi)
    """
    while th > np.pi:
        th -= 2 * np.pi
    while th < -np.pi:
        th += 2 * np.pi
    return th

#####################
# SPATIAL FUNCTIONS #
#####################


def is_near(pos, center, threshold=0.05):
    """
    Returns a true if the robot is inside a certain area determined by an xy center and a given radius.
    """
    if type(pos) != np.ndarray:     # TODO: cosas de tipo de dato y movidas asi y same es is_near_angle
        pos = [pos.x.value, pos.y.value]
    return math.pow(pos[0] - center[0], 2) + math.pow(pos[1] - center[1], 2) <= math.pow(threshold, 2)


def is_near_angle(robot_th, th, threshold=0.1):
    """ Returns True if the robot's angle matches that of the one specified in the th argument (with a default threshold value of 5.7º)"""
    if not isinstance(robot_th, (int, float)):
        robot_th = robot_th.th.value
    if abs(norm_pi(robot_th)-norm_pi(th)) < threshold:
        return True
    return (math.pi - abs(norm_pi(robot_th)) + math.pi - abs(norm_pi(th)) < threshold)


def absolute_offset(robot, distance=0):
    final_pos = np.array([robot.x.value + distance*math.sin(robot.th.value),
                         robot.y.value + distance*math.cos(robot.th.value)])
    #plt.arrow(robot.x.value, robot.y.value, math.cos(robot.th.value), math.sin(robot.th.value))
    #plt.plot(robot.x.value, robot.y.value, 'rx')
    #plt.plot(final_pos[0], final_pos[1], 'bx')
    # plt.show()
    return final_pos

####################
# VISION FUNCTIONS #
####################


def get_blob(frame):
    """ Searches for a blob and returns the center """
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #avg_brightness = np.average(np.linalg.norm(hsv, axis=2))
    #print('brightness: {}'.format(avg_brightness))
    # if avg_brightness > 140:
    #    diff = avg_brightness - 140
    #    for pixel in hsv:
    #        pixel[2] = pixel[2] - diff
    #avg_brightness = np.average(np.linalg.norm(hsv, axis=2))
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2
    res = cv.bitwise_and(frame, frame, mask=mask)   # we apply the mask...
    # detect the blobs (detector params are outside for performance)
    keypoints = detector.detect(res)
    if not keypoints:
        biggest_kp = None
    else:   # we only keep the biggest blob!
        biggest_kp = keypoints[0]
        for point in keypoints:
            if point.size > biggest_kp.size:
                biggest_kp = point
    return biggest_kp   # return the chonky one


def show_cam_blobs(robot):
    """Shows a video and marks the biggest blob if any are found"""
    while (True):
        tIni = time.clock()
        frame = robot.takePic()[139:179, 219:239]
        blob = get_blob(frame=frame)
        print('tiempo de procesado es: {}'.format(time.clock()-tIni))
        if blob:
            print(blob.size)
            im_with_keypoints = cv.drawKeypoints(frame, [blob], np.array(
                []), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            im_with_keypoints = frame
        cv.imshow('img', im_with_keypoints)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        tEnd = time.clock()
    cv.destroyAllWindows()

#################
# MAP FUNCTIONS #
#################

def draw_map(grid, direction = None, pos = None):
    """
    Prints the grid and, if given, the direction of the robot
    """
    lim_str = '+---' * len(grid[0, :]) + '+'
    arrow_list = ['↓', '←', '↑', '→']
    ascii_grid = grid.astype(int).tolist()
    if direction and pos:
        ascii_grid[(int(pos[0])), int(pos[1])] = arrow_list[direction]
    print(tabulate(ascii_grid, tablefmt='grid'))

def read_map(file):
    """This function returns the size and arrays of the given map.
    Size is given by a nxn map of tiles of a mm.
    The map represents the accessible points (0's beign accessible and 1's inaccessible)"""
    size = np.loadtxt(file, max_rows=1)
    size[2] = size[2]/1000
    map = np.loadtxt(file, dtype='int', skiprows=1)
    return size, map


def next_cell(grid, moves, offset_angle, arr_pos, smallest_value):
    for i in range(len(moves)):
        real_index = i+offset_angle  # the index corresponding to the non-relative values
        # if we surpassed the array's limits, loop through the begginning
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


# for now im considering the origin of coordinates in the map's array origin (map[0][0] = (0,0)) (top left!)
def pos2array(map_size, map, pos):
    """Turns real-world coordinates into their equivalent map positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the tiles border position on the map only if it matches exactly that position, otherwise it will return the tile position"""
    cell = np.array([0, 0])
    cell[0] = (pos[0] - map_size[2]/2) / map_size[2]
    cell[1] = (pos[1] - map_size[2]/2) / map_size[2]
    cell = tile2array(map_size, cell)
    cell = cell.astype(np.float32)
    return cell  # we could also return a modified map maybe with the -3 value in the given position? or the value of a map in that position


# for now im considering the origin of coordinates in the map's array origin map[0][0] = 0,0
def array2pos(map_size, map, cell):
    """Turns the map's coordinates into their real-world  positions in the array using said map's size.\n
    You can use the map's size vector or the tile size directly.\n
    The value will go to the middle of every tile or tile border"""
    pos = np.array([0, 0], dtype=np.float32)
    pos[0] = cell[0]/2 * map_size[2]
    pos[1] = (map_size[1]-(cell[1])/2)*map_size[2]
    return pos


def tile2array(size, their_coord):
    x = 2 * their_coord[0] + 1
    y = 2 * size[1] - 1 - 2 * their_coord[1]
    return np.array([x, y])
