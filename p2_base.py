#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
import movement as mv

def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)
        oscar = Robot()
        oscar.setup()
        oscar.startOdometry()
        #oscar.setSpeed(10, 0)
        #time.sleep(5)
        #oscar.setSpeed(0, 10)
        #time.sleep(5)
        #oscar.stopOdometry()
        #oscar.stopRobot()      
        mv.eight(oscar, 0.2, 1)

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



