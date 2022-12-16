import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.transforms as trans
import matplotlib.animation as FuncAnimation
import pandas as pd
import numpy as np
import helpers.map

def plot_file(file_name, map_name):
    size = helpers.map.read_map(map_name)[0]
    map = helpers.map.read_map(map_name)[1]
    size[2] *= 1000
    size = [int(param) for param in size]
    fig = plt.figure()
    mapLine = 'p-'
    df = pd.read_csv(file_name)
    ax = fig.add_subplot(111)   
    
    # Creamos una figura grid, que represente las paredes del mapa:
    plt.rc('grid', linestyle ="--", color='gray')
    plt.grid(True)
    plt.tight_layout()
    x_t = range(0, (size[0] + 1)* size[2], size[2]) #TODO: bajar las escalas del grid para que se vea cuadrado y coincida con las baldosas
    y_t = range(0, (size[1] + 1) * size[2], size[2])
    #x_labels = [str(n/1000) for n in x_t]
    #y_labels = [str(n/1000) for n in y_t]
    #plt.xticks(x_t, x_labels)
    #plt.yticks(y_t, y_labels)

    #the main frame of the map:
    X = np.array([0, size[0], size[0], 0, 0]) * size[2] / 1000 
    Y = np.array([0, 0, size[1], size[1], 0]) * size[2] / 1000
    base = plt.gca().transData
    rot = trans.Affine2D().rotate_deg(270)
    ax.plot(X, Y, mapLine)
    #vertical walls:
    for i in range(2, 2 * size[1], 2):
        for j in range(1, 2 * size[0], 2):
            if not map[i,j]:
                cx = np.floor((i-1)/2) - 5
                cy = np.floor((j-1)/2)
                X = np.array([cx + 1, cx + 1]) * size[2] / 1000
                Y = np.array([cy, cy + 1]) * size[2] / 1000
                ax.plot(X, Y, mapLine, transform = rot + base)
    #horizontal walls:
    for j in range(2, 2 * size[0], 2):
        for i in range(1, 2 * size[1], 2):
            if not map[i,j]:
                cx = np.floor((i-1)/2) - 5
                cy = np.floor((j-1)/2)
                X = np.array([cx, cx + 1]) * size[2] / 1000
                Y = np.array([cy + 1, cy + 1]) * size[2] / 1000
                ax.plot(X, Y, mapLine, transform = rot + base)
    ax.plot(df['x'], df['y']) 
    #plt.axis('equal')
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
