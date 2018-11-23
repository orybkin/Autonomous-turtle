#!/usr/bin/env python

from b3m33aro import Turtlebot
import numpy as np
import time
import cv2
from math import *

from utils.path_controller import PathController
from utils.rrt_planner import *
from utils.marker_registrator import *
from utils.map_registrator import *
from utils.robot import *

running = True
active = True

def click(vent, x, y, flags, param):
    global active
    active = not active
    print active


def main():
    time_zero=time.time()
    i=0
    period=100000;

    turtle = Turtlebot(True, True, True)
    turtle.reset_odometry()

    # wait for robot and get data
    while turtle.get_odometry() is None:
        print('NO ODOMETRY')

    while turtle.get_point_cloud() is None:
        print('NO PC')

    robot=Robot(turtle)
    #robot.make_plan([robot.CIRCLE, robot.PLAN_PATH, robot.GO])

    robot.look_around()
    #robot.register_obstacles()

    robot.estimate_scene_dimensions()

    robot.time_update=2

    robot.go_right_to_checkpoint()

    robot.go_right_to_checkpoint()

    robot.go_right_to_checkpoint()

    robot.go_right_to_checkpoint()

if __name__ == "__main__":
    main()
