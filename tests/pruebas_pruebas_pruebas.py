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
from tabulate import tabulate

goal = [4, 1]
direction = 2
pos = [1, 4]
[size, map] = helpers.map.read_map('maps/mapa3.txt')
goal = helpers.map.tile2array(size, goal)
grid = helpers.map.generate_grid(map, goal)  

####################################draw_map(grid, direction = None, pos = None):
print('Y \n↑ \no → X')
lim_str = '+---' * len(grid[0, :]) + '+'
arrow_list = ['↑', '→', '↓', '←']
#sustituimos las paredes por 'bloques' (ASCII 219) 
ascii_grid = grid.astype(int).tolist()
for j in range (len(grid[0, :])):
    for i in range (len(grid[:, 0])):
        if (grid[i,j] == -1):
            ascii_grid[i][j] = '█'
if (direction is not None) and (pos is not None):
    ascii_grid[int(pos[0])][int(pos[1])] = arrow_list[int(direction)]
print(tabulate(ascii_grid, tablefmt='grid'))
