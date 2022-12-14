# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for reading an NXT ultrasonic sensor connected to PORT_1 of the BrickPi3
# 
# Hardware: Connect an NXT ultrasonic sensor to BrickPi3 Port 1.
# 
# Results:  When you run this program, you should see the distance in CM.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import time     # import the time library for the sleep function
import libs.brickpi3 as brickpi3 # import the BrickPi3 drivers


BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

BP.reset_all()
# Configure for an NXT ultrasonic sensor.
# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.NXT_ULTRASONIC specifies that the sensor will be an NXT ultrasonic sensor.
li_sen = BP.PORT_3
BP.set_sensor_type(li_sen, BP.SENSOR_TYPE.NXT_LIGHT_ON)     #EV3_ULTRASONIC_CM)

try:
    while True:
        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).
        try:
            sen_value = BP.get_sensor(li_sen)                      # print the distance in CM
        except brickpi3.SensorError as error:
            print('error')
        else:
            print('value: {}'.format(sen_value))
        
        time.sleep(0.05)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware. https://github.com/DexterInd/BrickPi3
