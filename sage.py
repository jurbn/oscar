import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as FuncAnimation
import csv
import pandas as pd
import logging

def plot_robot(loc_eje, c, tamano):
    """
    Dibuja robot en location_eje con color (c) y tamano (p/g)
    """
    if tamano == 'p':
        largo = 0.1
        corto = 0.05
        descentre = 0.01
    else:
        largo = 0.5
        corto = 0.25
        descentre = 0.05

    trasera_dcha = np.array([-largo, -corto, 1])
    trasera_izda = np.array([-largo, corto, 1])
    delantera_dcha = np.array([largo, -corto, 1])
    delantera_izda = np.array([largo, corto, 1])
    frontal_robot = np.array([largo, 0, 1])
    tita = loc_eje[2]
    Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
                    [np.sin(tita), np.cos(tita), loc_eje[1]],
                    [0,        0,        1]])
    Hec = np.array([[1, 0, descentre],
                    [0, 1, 0],
                    [0, 0, 1]])
    extremos = np.array([trasera_izda, delantera_izda, delantera_dcha,
                        trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))
    plt.plot(robot[0, :], robot[1, :], c)

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


def is_near(location, center, radius):
    """
    Returns a true if the robot is inside a certain area determined by an xy center and a given radius.
    """
    robot_x = location[0]
    robot_y = location[1]
    center_x = center[0]
    center_y = center[1]
    return math.pow(robot_x - center_x, 2) + math.pow(robot_y - center_y, 2) <= math.pow(radius, 2)

def enc_to_speed(robot, n_values, p_values, time):
    v = 0
    w = 0
    return v, w
