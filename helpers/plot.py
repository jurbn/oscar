import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation
import pandas as pd
import numpy as np
import helpers.map
import logging

def plot_file(robot):  #TODO: guardar dónde acaba cada etapa para plotearlo en diferentes colores
    size_0 = int(robot.map_size[0])
    size_1 = int(robot.map_size[1])   # sizes of the map
    tile_size = robot.map_size[2]
    grid = get_grid(robot, 2)

    f = plt.figure()
    fig = f.add_subplot(111)   # subplot where we will draw
    
    # plot config:
    plt.grid(True)
    plt.xticks(np.arange(0, (size_0 + 1)* tile_size, tile_size)) #we set the axis ticks to match the tiles
    plt.yticks(np.arange(0, (size_1 + 1) * tile_size, tile_size))
    plt.gca().set_aspect("equal")
    colors = ['tomato', 'orange', 'gold', 'yellow','greenyellow', 'aquamarine', 'cornflowerblue', 'mediumorchid']
    # obstacles:
    for i in range (size_1 * 2 + 1):
        for j in range(size_0 * 2 + 1):
            if (grid[i,j] == -1): #if the map cell is zero, it means there's an obstacle:
                if (i%2==0) and (j%2==1): #horizontal wall
                    #logging.debug('pared horizontal en: [{}, {}]'.format(i,j)) 
                    [posx, posy] = helpers.map.array2pos(robot.map_size, [i,j])
                    X = np.array([posx - tile_size/2, posx + tile_size/2])
                    Y = np.array([posy, posy])
                    fig.plot(X, Y, linewidth = '2', color = colors[i//2]) 
                elif (i%2==1) and (j%2==0): #vertical wall
                    #logging.debug('pared vertical en: [{}, {}]'.format(i,j)) 
                    [posx, posy] = helpers.map.array2pos(robot.map_size, [i,j])
                    X = np.array([posx, posx])
                    Y = np.array([posy - tile_size/2, posy + tile_size/2])
                    fig.plot(X, Y, linewidth = '2', color = colors[i//2]) 
                elif (i%2==1) and (j%2==1): #obstá([culo]) (.)(.)
                    #logging.debug('columna en: [{}, {}]'.format(i,j)) 
                    [X, Y] = helpers.map.array2pos(robot.map_size, [i,j])
                    fig.plot(X, Y, marker='8', markersize = 15, color = 'pink') 
    
    # objective(s):
    if robot.objective:
        try:
            for obj in robot.objective:
                X = helpers.map.tile2pos(robot.map_size, obj)[0]
                Y = helpers.map.tile2pos(robot.map_size, obj)[1]    
                fig.plot(X, Y, marker = "x", color = 'greenyellow')
        except Exception:
            [X, Y] = helpers.map.tile2pos(robot.map_size, robot.objective)
            fig.plot(X, Y, marker = "x", color = 'greenyellow') #TODO: poner colores decentes para los objetivos
    
    # ball:
    if robot.ball_caught_in:    # if the robot has caught the ball, mark it with a red circle
        [X, Y] = robot.ball_caught_in
        fig.plot(X, Y, 'r', marker = "o", markersize = 15)

    # odometry: 
    df = pd.read_csv(robot.odometry_file, skiprows = [1,2,3])   # the dataframe where the odometry values are stored
    fig.plot(df['x'], df['y'], color = 'cornflowerblue') 

    plt.show()    

def get_grid(robot, columns):
    i = 1
    j = 1
    n = 0
    grid = robot.grid_plot
    finished = False
    while (i < (robot.map_size[1] * 2 + 1)) and not finished:
        while (j < (robot.map_size[0] * 2 + 1)) and not finished:
            if (robot.grid[i, j] == -1):
                n += 1
                grid[i + 1, j] = 1
                grid[i, j + 1] = 1
                grid[i - 1, j] = 1
                grid[i, j - 1] = 1
                if n == columns: finished = True
            j += 2
        j = 1
        i += 2
    return grid

def plot_animation(robot):  #FIXME: unused, right?
    fig = plt.figure(figsize=(6, 3))
    x = [0]
    y = [0]
    ln, = plt.plot(x, y, '-')
    def update_plot(frame):
        x.append(robot.x.value)
        y.append(robot.y.value)
        ln.set_data(x, y)
        fig.gca().relim()
        fig.gca().autoscale_view()
        return ln,
    animation = FuncAnimation(fig, update_plot, interval=50)
    plt.show()
