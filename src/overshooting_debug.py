#!/usr/bin/env python

from b3m33aro import Turtlebot
import numpy as np
import time
import cv2
from math import *
from utils.path_controller import *
from utils.map_registrator import *
from utils.path_controller_PID import *
from utils.robot import *


WINDOW = 'go path'

running = True
active = True


def click(vent, x, y, flags, param):
    global active
    active = not active
    print active


def main():
    turtle = Turtlebot(True, True, True)
    turtle.reset_odometry()

    # wait for robot and get data
    while turtle.get_odometry() is None:
        print('NO ODOMETRY')

    while turtle.get_point_cloud() is None:
        print('NO PC')

    robot = Robot(turtle)

    robot.register_obstacles()

    robot.go_right_to_checkpoint()


if __name__ == "__main__":
    main()
