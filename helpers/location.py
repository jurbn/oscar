import math
import logging
import numpy as np

import helpers

def get_angle_between_points(first, second):
    """Returns the absolute angle between two points"""
    angle = math.atan2(second[1] - first[1], second[0] - first[0])
    angle = helpers.maths.norm_pi(angle)
    return angle

def get_distance_between_points(first, second):
    """Returns the distance between two points"""
    part_one = math.pow(second[0] - first[0], 2)
    part_two = math.pow(second[1] - first[1], 2)
    distance = math.sqrt(part_one + part_two)
    return distance

def is_near(pos, center, threshold=0.05):
    """
    Returns a true if the robot is inside a certain area determined by an xy center and a given radius.
    """
    if (type(pos) != np.ndarray) and (type(pos) != list):
        pos = [pos.x.value, pos.y.value]
    return math.pow(pos[0] - center[0], 2) + math.pow(pos[1] - center[1], 2) <= math.pow(threshold, 2)


def is_near_angle(robot_th, th, threshold=0.05):
    """ Returns True if the robot's angle matches that of the one specified in the th argument (with a default threshold value of 5.7º)"""
    if not isinstance(robot_th, (int, float)):
        robot_th = robot_th.th.value
    if abs(helpers.maths.norm_pi(robot_th)-helpers.maths.norm_pi(th)) < threshold:
        return True
    return (math.pi - abs(helpers.maths.norm_pi(robot_th)) + math.pi - abs(helpers.maths.norm_pi(th)) < threshold)


def absolute_offset(robot, distance=0):
    """Returns the position of a point at a certain distance in front of the robot"""
    final_pos = np.array([robot.x.value + distance*math.sin(robot.th.value),
                         robot.y.value + distance*math.cos(robot.th.value)])
    return final_pos

def get_robot_quadrant(robot, index = False):
    """Returns the nearest n*pi/2"""
    th = robot.th.value
    ob_th = th
    if is_near_angle(th, 0, threshold=math.pi/4):
        if index:
            ob_th = 1
        else:
            ob_th = 0
    elif is_near_angle(th, math.pi/2, threshold=math.pi/4):
        if index:
            ob_th = 0
        else:
            ob_th = math.pi/2
    elif is_near_angle(th, math.pi, threshold=math.pi/4):
        if index:
            ob_th = 3
        else:
            ob_th = math.pi
    elif is_near_angle(th, -math.pi/2, threshold=math.pi/4):
        if index:
            ob_th = 2
        else:
            ob_th = -math.pi/2
    return ob_th

def print_angle(th, deg = False):
    """turns any given angle in radians to a friendly format for printing, can also convert radians to deg"""
    if deg:
        return ('{}º'.format(math.degrees(th)))
    else:
        return ('{} * π rad'.format(th/math.pi)) 
