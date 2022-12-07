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
pos = [0, 4]
[size, map] = helpers.map.read_map('maps/mapa3.txt')
goal = helpers.map.tile2array(size, goal)
grid = helpers.map.generate_grid(map, goal)  
####################################draw_map(grid, direction = None, pos = None):
print('o → Y \n↓ \nX')
lim_str = '+---' * len(grid[0, :]) + '+'
arrow_list = ['↑', '→', '↓', '←']
#sustituimos las paredes por 'bloques' (ASCII 219) TODO: AÑADIR!!
ascii_grid = grid.astype(int).tolist()
if (direction is not None) and (pos is not None):
    ascii_grid[int(pos[0])][int(pos[1])] = arrow_list[int(direction)]
print(tabulate(ascii_grid, tablefmt='grid'))
#sustituimos las paredes por 'bloques' (ASCII 219) TODO: AÑADIR!!