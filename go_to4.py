import numpy as np
import math
import sage

def go_to(robot, pos, v= 0.1):
    og_pos = [robot.x.value, robot.y.value, robot.th.value] #posicion inicial del robot en su referencia
    x = pos[0]-og_pos[0]
    y = pos[1]-og_pos[1] #la distancia que tiene que recorrer el robot hasta el objetivo pos
    th = np.arctan2(y, x) #el ángulo que tiene que girar hasta pos
    th_spun = 0
    if abs(th) > math.pi/4:
        while abs(th - th_spun) < 0.08: 
            robot.setSpeed(0,(th/abs(th))*w)
            if abs(robot.th.value - th_p) < math.pi/2: #básicamente si no ha normalizado:
                th_spun = robot.th.value - th_p #actualizamos con la diferencia
            th_p = robot.th.value #en cualquier caso actualizamos
    # ahora va a estar siempre mirando hacia la casilla a la que tiene que moverse así que tiene que moverse en línea recta hasta pos
    while not sage.is_near(robot, pos, threshold=0.1):
        #print(pos)
        #print(robot.x.value, robot.y.value)
        robot.setSpeed(v,0);
    return True
