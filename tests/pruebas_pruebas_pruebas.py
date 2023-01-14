import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation


######################################ploteo weno
def plot():
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
    size_0 = 7
    size_1 = 7
    tile_size = 0.4
    map_size = [size_0, size_1, tile_size]

    fig = plt.figure()
    #df = pd.read_csv(robot.odometry_file, skiprows = [1,2,3])   # the dataframe where the odometry values are stored
    plot = fig.add_subplot(111)   # subplot where we will draw

    # obstacles:
    for i in range (size_1 * 2):
        for j in range(size_0 *2):
            if not map[i,j]: #if the map cell is zero, it means there's a wall:
                if (i%2==0) and (j%2==1): #horizontal wall
                    print('pared horizontal en: [{}, {}]'.format(i,j)) 
                    [posx, posy] = array2pos(map_size, [i,j])
                    X = np.array([posx - tile_size/2, posx + tile_size/2])
                    Y = np.array([posy, posy])
                    plot.plot(X, Y) 
                elif (i%2==1) and (j%2==0): #vertical wall
                    print('pared vertical en: [{}, {}]'.format(i,j)) #AQUIvvvvvv
                    [posx, posy] = array2pos(map_size, [i,j])
                    X = np.array([posx, posx])
                    Y = np.array([posy - tile_size/2, posy + tile_size/2])
                    plot.plot(X, Y) 
                elif (i%2==1) and (j%2==1): #obstá([culo])
                    print('columna en: [{}, {}]'.format(i,j)) 
                    [X, Y] = array2pos(map_size, [i,j])
                    plot.plot(X, Y) 
    
    #plot.plot(df['x'], df['y']) 
    plt.xticks(np.arange(0, (size_0+1)*tile_size, tile_size))
    plt.yticks(np.arange(0, (size_1+1)*tile_size, tile_size))
    #plt.gca().set_aspect("equal")
    plt.show()    


def array2pos(map_size, cell): 
    pos = np.array([0, 0], dtype=float)
    pos[0] = cell[1] * (map_size[2]/2)
    pos[1] = (map_size[1]*2-cell[0]) * (map_size[2]/2)
    return pos
