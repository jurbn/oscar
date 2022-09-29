from __future__ import print_function, division

import time
import brickpi3

BP = brickpi3.BrickPi3()
M_RI = BP.PORT_A
M_LE = BP.PORT_B
M_CL = BP.PORT_C

def  do_a_wheelie(speed):
    BP.set_motor_power(M_LE + M_RI, speed)

def grab_em(pos):
    BP.set_motor_position(M_CL, pos)

def main():
    BP.set_motor_limits(M_CL, 50, 60)
    BP.set_motor_limits(M_LE, 50, 60)
    BP.set_motor_limits(M_RI, 50, 60)
    grab_em(-25)
    time.sleep(7)
    grab_em(200)
    do_a_wheelie(0)

if __name__ == '__main__':
    main()
