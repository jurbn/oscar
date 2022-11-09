from __future__ import print_function # use python 3 syntax but make it compatible with python 2
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

# Parameter dealing n stuff
params = cv.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 100
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
    print('Holi soy el metodo hom y no estoy terminado :(')


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


def is_near(robot, center, threshold):
    """
    Returns a true if the robot is inside a certain area determined by an xy center and a given radius.
    """
    return math.pow(robot.x.value - center[0], 2) + math.pow(robot.y.value - center[1], 2) <= math.pow(threshold, 2)

def absolute_offset(robot, distance = 0):
    return np.array([robot.x.value + distance*math.cos(robot.th.value), robot.y.value + distance*math.sin(robot.th.value)])


def get_blob(frame):
    """ Searches for a blob and returns the center """
    # Parameter dealing n stuff
#    params = cv.SimpleBlobDetector_Params()
#    params.minThreshold = 10
#    params.maxThreshold = 200
#    params.filterByArea = True
#    params.minArea = 200
#    params.maxArea = 10000
#    params.filterByCircularity = True
#    params.minCircularity = 0.1
#    params.maxCircularity = 1
#    params.filterByColor = False
#    params.filterByConvexity = False
#    params.filterByInertia = False
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2
    # copiado de internet no tengo ni idea la verdad
    res = cv.bitwise_and(frame, frame, mask=mask)
    keypoints = detector.detect(res)
    if not keypoints:
        biggest_kp = None
    # The only blob we want is the biggest one (chonk)
    else:
        biggest_kp = keypoints[0]
        for point in keypoints:
            if point.size > biggest_kp.size:
                biggest_kp = point
    return biggest_kp

def show_cam_blobs(robot):
    """Shows a video and marks the blobs when found"""
    while(True):
        tIni = time.clock()
        frame = robot.takePic()
        blob = get_blob(frame=frame)
        print('tiempo de procesado es: {}'.format(time.clock()-tIni))
        if blob:
            print(blob.size)
            im_with_keypoints = cv.drawKeypoints(frame, [blob], np.array([]), (255,255,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            im_with_keypoints = frame
        cv.imshow('img', im_with_keypoints)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        tEnd = time.clock()
    cv.destroyAllWindows()
