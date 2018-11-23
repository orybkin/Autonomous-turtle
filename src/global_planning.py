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

    robot.go_checkpoint()

    robot.register_obstacles()

    robot.go_checkpoint()

    robot.register_obstacles()

    robot.go_checkpoint()

    robot.register_obstacles()

    robot.go_checkpoint()

    # ## go
    # while not turtle.is_shutting_down():
    #
    #     odom = turtle.get_odometry()
    #
    #     i += 1;
    #     if i == period :
    #         i = 0
    #         time_one = time.time()
    #         print(str(time_one - time_zero) + ' elapsed in ' + str(period) + ' cycles')
    #         time_zero = time_one
    #         print('we are at ' + str(odom[0:2]) + ', bearing ' + str(odom[2]))
    #         print ('start odometry is ' + str(robot.path_controller.start_odom[0:3]))
    #         print ('angles difference is ' + str(odom[2] - robot.path_controller.start_odom[2]))
    #
    #     robot.frame_update(odom)

if __name__ == "__main__":
    main()
