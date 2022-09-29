from __future__ import print_function, division

import time
import brickpi3
import read

BP = brickpi3.BrickPi3()
M_RI = BP.PORT_A
M_LE = BP.PORT_B
M_CL = BP.PORT_C

def do_a_wheelie(v, w):
    """
    Sets the power needed to achieve the given linear and angular speeds
    """
    # calculate the power for each motor
    p_le = 0
    p_ri = 0
    # apply that power 
    BP.set_motor_power(M_LE, p_le)
    BP.set_motor_power(M_RI, p_ri)

def open_em():
    """
    Sets the claw on its open state
    """
    BP.set_motor_position(M_CL, 200)

def close_em():
    """
    Sets the claw on its close state
    """
    BP.set_motor_position(M_CL, -25)

def main():
    BP.set_motor_limits(M_CL, 50, 60)
    BP.set_motor_limits(M_LE, 50, 60)
    BP.set_motor_limits(M_RI, 50, 60)
    close_em()
    time.sleep(7)
    open_em()
    do_a_wheelie(10, 0)

if __name__ == '__main__':
    main()
