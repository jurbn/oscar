import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation
import random

map = np.array([[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                [0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
cols = 2
size_0 = 7
size_1 = 7
tile_size = 0.4
map_size = [size_0, size_1, tile_size]

def plot(map, map_size):

    p = plt.figure()
    fig = p.add_subplot()
    #df = pd.read_csv(robot.odometry_file, skiprows = [1,2,3])   # the dataframe where the odometry values are stored
    plt.xticks(np.arange(0, (size_0+1)*tile_size, tile_size))
    plt.yticks(np.arange(0, (size_1+1)*tile_size, tile_size))
    plt.grid(True)
    plt.gca().set_aspect("equal")
    colors = ['tomato', 'orange', 'gold', 'yellow','greenyellow', 'aquamarine', 'cornflowerblue', 'mediumorchid']
    print('COLORS LENGTH: ',len(colors))
    # obstacles:
    for i in range (size_1 * 2 + 1):
        print ('i//2 = ', i//2)
        for j in range(size_0 *2 + 1):
            if not map[i,j]: #if the map cell is zero, it means there's a wall:
                if (i%2==0) and (j%2==1): #horizontal wall
                    [posx, posy] = array2pos(map_size, [i,j])
                    X = np.array([posx - tile_size/2, posx + tile_size/2])
                    Y = np.array([posy, posy])
                    print(i)
                    fig.plot(X, Y, linewidth = '2', color = colors[i//2]) 
                elif (i%2==1) and (j%2==0): #vertical wall
                    [posx, posy] = array2pos(map_size, [i,j])
                    X = np.array([posx, posx])
                    Y = np.array([posy - tile_size/2, posy + tile_size/2])
                    fig.plot(X, Y, linewidth = '2', color = colors[i//2]) 
                elif (i%2==1) and (j%2==1): #obstá([culo])
                    [X, Y] = array2pos(map_size, [i,j])
                    fig.plot(X, Y, marker='8', markersize = 15, color = 'pink') 
    
    #plot.plot(df['x'], df['y']) 
    plt.show()    

def get_color():
    colors = ['lawngreen', 'palegreen', 'springgreen','greenyellow', 'lightgreen', 'lime', 'mediumspringgreen']
    return random.choice(colors)


def rmv_col_walls(map, map_size, columns):# removing the walls around the columns: TODO: puedo hacer un metodo de esto con el número de columnas that'd be hot
    i = 1
    j = 1
    n = 0
    finished = False
    while (i < (map_size[1] * 2 + 1)) and not finished:
        while (j < (map_size[0] * 2 + 1)) and not finished:
            if not map[i, j]:
                n += 1
                map[i + 1, j] = 1
                map[i, j + 1] = 1
                map[i - 1, j] = 1
                map[i, j - 1] = 1
                if n == columns: finished = True
            j += 2
        j = 1
        i += 2
    return map


def array2pos(map_size, cell): 
    pos = np.array([0, 0], dtype=float)
    pos[0] = cell[1] * (map_size[2]/2)
    pos[1] = (map_size[1]*2-cell[0]) * (map_size[2]/2)
    return pos

plot(rmv_col_walls(map, map_size, cols), map_size)