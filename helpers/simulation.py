import numpy as np
import helpers.maths

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
        x_r_r2 = np.array([r*helpers.maths.chord(th)*np.sin(al), r*helpers.maths.chord(th)*np.cos(al), th])
    x_w_r2 = helpers.maths.loc(np.matmul(helpers.maths.hom(x_w_r), helpers.maths.hom(x_r_r2)))   # new location x_w_r
    return x_w_r2
