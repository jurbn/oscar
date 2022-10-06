#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        oscar = Robot()

        print("X value at the beginning from main X= %.2f" %(oscar.x.value))

        # 1. launch updateOdometry Process()
        oscar.startOdometry()

        # 2. perform trajectory


        # DUMMY CODE! delete when you have your own
        oscar.setSpeed(1,1)
        print("Start : %s" % time.ctime())
        time.sleep(3)
        print("X value from main tmp %d" % oscar.x.value)
        time.sleep(3)
        print("End : %s" % time.ctime())

        oscar.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (oscar.x.value, oscar.y.value, oscar.th.value))
        oscar.lock_odometry.release()

        # PART 1:
        # oscar.setSpeed()
        # until ...

        # PART 2:
        # oscar.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        oscar.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        oscar.stopOdometry()
        oscar.stopRobot()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



