import numpy as np
import matplotlib.pyplot as plt


def dibrobot(loc_eje, c, tamano):
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


def simubot(vc, xWR, t):
    """
    Simula movimiento del robot con vc=[v,w] en T seg. desde xWR
    """
    if vc[1] == 0:   # w=0
        xRk = np.array([vc[0]*t, 0, 0])
    else:
        R = vc[0]/vc[1]
        dtitak = vc[1]*t
        titak = np.norm_pi(dtitak)
        xRk = np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])

    xWRp = loc(np.dot(hom(xWR), hom(xRk)))   # nueva localizacion xWR
    return xWRp

def hom(x: np.array):
    """
    Returns the Transformation matrix based on the location vector
    """
    return np.matrix([[np.cos(x[2]), -np.sin(x[2]) , x[0]], [np.sin(x[2]), np.cos(x[2]), x[1]], [0, 0, 1]])


def loc(T):
    """
    Returns the location vector based on the transformation matrix
    """
    return np.array([T.item(0, 2), T.item(1, 2) , np.arctan2(T.item(1, 0), T.item(0, 0))])
