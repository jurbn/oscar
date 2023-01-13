import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation
import pandas as pd
import numpy as np
import helpers.map

def plot_file(robot):  #TODO: distintos formatos del csv para las distintas etapas
    size = helpers.map.read_map(robot.map_file)[0]
    map = helpers.map.read_map(robot.map_file)[1]
    size[2] *= 1000
    size = [int(param) for param in size]
    fig = plt.figure()
    df = pd.read_csv(robot.odometry_file, skiprows = [1,2,3])
    ax = fig.add_subplot(111)   

    # Creamos una figura grid, que represente las paredes del mapa:
    plt.rc('grid', linestyle ="--", color='gray')
    plt.grid(True)
    plt.tight_layout()
    x_t = range(0, (size[0] + 1)* size[2], size[2]) 
    y_t = range(0, (size[1] + 1) * size[2], size[2])

    #the main frame of the map:
    X = np.array([0, size[0], size[0], 0, 0]) * size[2] / 1000 
    Y = np.array([0, 0, size[1], size[1], 0]) * size[2] / 1000
    base = plt.gca().transData
    rot = trans.Affine2D().rotate_deg(270)
    ax.plot(X, Y)

    #figuras extra:
    grid = robot.grid
    for i in range(1, 2*size[0], 2):
        for j in range(1, 2*size[0], 2):
            if not robot.grid[i, j]: #Goal position, marked with an x
                [X, Y] = np.array(helpers.map.array2pos(size, map, [i, j]))/1000
                ax.plot(X, Y, marker = "x")
            if robot.grid[i,j] == -1: #Slalom obstacles, marked with octagons
                robot.grid[i+1,j] = 1
                robot.grid[i-1,j] = 1
                robot.grid[i,j-1] = 1
                robot.grid[i,j+1] = 1
                [X, Y] = np.array(helpers.map.array2pos(size, map, [i, j]))/1000
                ax.plot(X, Y, marker = "8", markersize = 15)
    if robot.ball_caught_in:
        [X, Y] = robot.ball_caught_in
        ax.plot(X, Y, 'r', marker = "o", markersize = 15)

    #vertical walls:
    for i in range(2, 2 * size[1], 2):
        for j in range(1, 2 * size[0], 2):
            if (grid[i,j] == -1):
                cx = np.floor((i-1)/2) - size[1]
                cy = np.floor((j-1)/2)
                X = np.array([cx + 1, cx + 1]) * size[2] / 1000
                Y = np.array([cy, cy + 1]) * size[2] / 1000
                ax.plot(X, Y, transform = rot + base) 
    
    #horizontal walls:
    for j in range(2, 2 * size[0], 2):
        for i in range(1, 2 * size[1], 2):
            if (grid[i,j] == -1):
                cx = np.floor((i-1)/2) - size[1]
                cy = np.floor((j-1)/2)
                X = np.array([cx, cx + 1]) * size[2] / 1000
                Y = np.array([cy + 1, cy + 1]) * size[2] / 1000
                ax.plot(X, Y, transform = rot + base) 
    ax.plot(df['x'], df['y']) 
    plt.xticks(np.arange(0, (size[0]+1)*0.4, 0.4))
    plt.yticks(np.arange(0, (size[1]+1)*0.4, 0.4))
    plt.gca().set_aspect("equal")
    plt.show()    


def plot_animation(robot):
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
