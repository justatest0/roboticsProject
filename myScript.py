#! /usr/bin/env python3
"""
Test client for the <eduMorse> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""

import sys
import json
import datetime
import time
import math
import numpy as np

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)


def trilateration(pose, prox, motion):

	# Getting coordinates from point 1 and the distance from Goal
    x1 = pose.get()['x']                        # get the current x coordinate
    y1 = pose.get()['y']                        # get the current y coordinate
    d1 = prox.get()['near_objects']['GOAL']     # get the current distance from goal

    # Moving to a point 2 and geting it's coordinates and distance from Goal
    motion.publish({"v": 0,   "w": 0.5})        # set robot angular speed
    time.sleep(1)                               # let the robot rotate
    motion.publish({"v": 0.5, "w": 0})          # set robot linear speed
    time.sleep(1)                               # let the robot move
    motion.publish({"v": 0,   "w": 0})          # stop the robot

    x2 = pose.get()['x']                        # get the current x coordinate
    y2 = pose.get()['y']                        # get the current y coordinate
    d2 = prox.get()['near_objects']['GOAL']     # get the current distance from goal

    # Moving to a point 3 and geting it's coordinates and distance from Goal
    motion.publish({"v": 0,   "w": -0.5})       # set robot angular speed
    time.sleep(1)                               # let the robot rotate
    motion.publish({"v": 0.5, "w": 0})          # set robot linear speed
    time.sleep(1)                               # let the robot move
    motion.publish({"v": 0,   "w": 0})          # stop the robot

    x3 = pose.get()['x']                        # get the current x coordinate
    y3 = pose.get()['y']                        # get the current y coordinate
    d3 = prox.get()['near_objects']['GOAL']     # get the current distance from goal

    # From 3 equations to 2 equations
    A = 2*(x2-x1)
    B = 2*(y2-y1)
    C = d1*d1 - d2*d2 - x1*x1 + x2*x2 - y1*y1 + y2*y2
    D = 2*(x3-x2)
    E = 2*(y3-y2)
    F = d2*d2 - d3*d3 - x2*x2 + x3*x3 - y2*y2 + y3*y3

    # Solve the 2 equation/2 unknown system using Cramer's criterion
    det   = A*E - B*D
    xGoal = (C*E - B*F)/ det
    yGoal = (A*F - C*D)/ det

    relOrient = math.acos(math.fabs(xGoal-x3)/d3)

    # Return the Goal's coordinates and relative orientation
    return [xGoal, yGoal,  d3, relOrient]


with Morse() as simu:

    # Definition of the sensors and the actuator
    pose = simu.our_robot.pose                  # pose sensor
    prox = simu.our_robot.prox                  # proximity sensor
    irFront = simu.our_robot.ir1                    # ir sensor #1
    irLeft = simu.our_robot.ir2                    # ir sensor #2
    irRight = simu.our_robot.ir3                    # ir sensor #3
    irRear = simu.our_robot.ir4                    # ir sensor #4
    motion = simu.our_robot.motion              # motion speed actuator


    # Find the Goal's coordinates and relative orientation
    print('Trilateration Mode - Finding the Goal')
    goalAt = trilateration(pose, prox, motion)
    print('Trilateration Mode - Goal Found at:', goalAt[0], goalAt[1])

    print("commettere")
