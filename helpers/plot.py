import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation
import pandas as pd
import numpy as np
import helpers.map
import logging

def plot_file(robot):  #TODO: guardar dónde acaba cada etapa para plotearlo en diferentes colores
    size = helpers.map.read_map(robot.map_file)[0] #TODO: cambiar al size de robot
    size_0 = int(size[0])
    size_1 = int(size[1])   # sizes of the map
    tile_size = size[2]
    fig = plt.figure()
    df = pd.read_csv(robot.odometry_file, skiprows = [1,2,3])   # the dataframe where the odometry values are stored
    ax = fig.add_subplot(111)   # subplot where we will draw

    # We create a grid figure that matches the floor tiles:
    plt.rc('grid', linestyle ="--", color='gray')
    plt.grid(True)
    plt.tight_layout()
    x_t = np.arange(0, (size_0 + 1)* tile_size, tile_size) 
    y_t = np.arange(0, (size_1 + 1) * tile_size, tile_size)

    # the main frame of the map:
    # X = np.array([0, size_0, size_0, 0, 0]) * tile_size  
    # Y = np.array([0, 0, size_1, size_1, 0]) * tile_size 
    base = plt.gca().transData
    rot = trans.Affine2D().rotate_deg(270)
    #ax.plot(X, Y)

    # extra figures:
    for i in range(1, 2*(size_0), 2):
        for j in range(1, 2*size_0, 2):
            if not robot.map[i,j]: #Slalom obstacles, marked with octagons
                robot.map[i+1,j] = 1
                robot.map[i-1,j] = 1
                robot.map[i,j-1] = 1
                robot.map[i,j+1] = 1
                [X, Y] = np.array(helpers.map.array2pos(size, robot.map, [i, j]))
                ax.plot(X, Y, marker = "8", markersize = 15)
    if robot.objective: # if the robot has a declared objective, mark it with an X
        try:
            for obj in robot.objective:
                X = helpers.map.tile2pos(robot.map_size, obj)[0]
                Y = helpers.map.tile2pos(robot.map_size, obj)[1]    
                ax.plot(X, Y, marker = "x")
        except Exception:
            [X, Y] = helpers.map.tile2pos(robot.map_size, robot.objective)
            ax.plot(X, Y, marker = "x")
    if robot.ball_caught_in:    # if the robot has caught the ball, mark it with a red circle
        [X, Y] = robot.ball_caught_in
        ax.plot(X, Y, 'r', marker = "o", markersize = 15)

    #vertical walls: EDIT theyre horizontal ITHINK
    for i in np.arange(0, 2 * size_1 + 1, 2):
        for j in np.arange(1, 2 * size_0 + 1, 2):
            if not robot.map[i,j]:
                cx = np.floor((i-1)/2) - size_1
                cy = np.floor((j-1)/2)
                X = np.array([cx + 1, cx + 1]) * tile_size
                Y = np.array([cy, cy + 1]) * tile_size
                ax.plot(X, Y, transform = rot + base) 
    
    #horizontal walls:
    for j in np.arange(0, 2 * size_0 + 1, 2):
        for i in np.arange(1, 2 * size_1 + 1, 2):
            if not robot.map[i,j]:
                cx = np.floor((i-1)/2) - size_1
                cy = np.floor((j-1)/2)
                X = np.array([cx, cx + 1]) * tile_size
                Y = np.array([cy + 1, cy + 1]) * tile_size
                ax.plot(X, Y, transform = rot + base) 
    ax.plot(df['x'], df['y']) 
    plt.xticks(np.arange(0, (size_0+1)*0.4, 0.4))
    plt.yticks(np.arange(0, (size_1+1)*0.4, 0.4))
    plt.gca().set_aspect("equal")
    plt.show()    


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
