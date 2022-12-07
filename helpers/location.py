import math
import logging
import numpy as np

import helpers

def is_near(pos, center, threshold=0.05):
    """
    Returns a true if the robot is inside a certain area determined by an xy center and a given radius.
    """
    if type(pos) != np.ndarray:     # TODO: cosas de tipo de dato y movidas asi y same es is_near_angle
        pos = [pos.x.value, pos.y.value]
    return math.pow(pos[0] - center[0], 2) + math.pow(pos[1] - center[1], 2) <= math.pow(threshold, 2)


def is_near_angle(robot_th, th, threshold=0.1):
    """ Returns True if the robot's angle matches that of the one specified in the th argument (with a default threshold value of 5.7º)"""
    if not isinstance(robot_th, (int, float)):
        robot_th = robot_th.th.value
    if abs(helpers.maths.norm_pi(robot_th)-helpers.maths.norm_pi(th)) < threshold:
        return True
    return (math.pi - abs(helpers.maths.norm_pi(robot_th)) + math.pi - abs(helpers.maths.norm_pi(th)) < threshold)


def absolute_offset(robot, distance=0):
    final_pos = np.array([robot.x.value + distance*math.sin(robot.th.value),
                         robot.y.value + distance*math.cos(robot.th.value)])
    #plt.arrow(robot.x.value, robot.y.value, math.cos(robot.th.value), math.sin(robot.th.value))
    #plt.plot(robot.x.value, robot.y.value, 'rx')
    #plt.plot(final_pos[0], final_pos[1], 'bx')
    # plt.show()
    return final_pos