import numpy as np

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

def get_sign(number):
    if number > 0:
        return 1
    elif number < 0:
        return -1
    else:
        return 0