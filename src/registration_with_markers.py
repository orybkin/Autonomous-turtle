#!/usr/bin/env python


# Script for analysis of the turtlebot behaviour:
# find out frequency of the main cycle
# find out odometry measure
# check odometry and SfM correctness
# check the script correctness (optional)

from b3m33aro import Turtlebot, detector
import numpy as np
import cv2
import time
from math import *
from utils.map_registrator import MapRegistrator
from utils.marker_registrator import MarkerRegistrator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

MOVE = 1
ROTATE = 2
STOP = 3
linear_vel = 0.2
angular_vel = 0.5
# linear_vel = 1
# angular_vel = 1
WINDOW = 'obstacles'
WINDOW_bird = 'birdview'
WINDOW_rgb = 'rgb'
running = True
active = True

SHOW_RGB = False


def state_str(state):
    if state == STOP:
        return ("STOP")
    elif state == ROTATE:
        return ("ROTATE")
    elif state == MOVE:
        return ("MOVE")


def click(vent, x, y, flags, param):
    if vent != 1:
        return
    global state
    global direction
    if y < 100:
        state = MOVE
    elif y < 260:
        state = ROTATE
        if x > 240:
            direction = -1
        else:
            direction = +1
    else:
        state = STOP
    print('coordinates ' + str(x) + ' ' + str(y))
    print('robot is going to ' + state_str(state))


def main():
    global state
    global direction
    state = STOP
    turtle = Turtlebot(rgb=True, pc=True, depth=True)
    turtle.reset_odometry()
    direction = None

    cv2.namedWindow(WINDOW)
    cv2.namedWindow(WINDOW_bird)
    cv2.setMouseCallback(WINDOW, click)
    cv2.moveWindow(WINDOW, 30, 30)
    cv2.moveWindow(WINDOW_bird, 600, 100)
    if SHOW_RGB:
        cv2.namedWindow(WINDOW_rgb)

    # used for testing
    time_prev = time_zero = time.time()
    i = 0
    period = 10;

    turtle.reset_odometry()
    time.sleep(1)
    reg = MapRegistrator()
    markreg=MarkerRegistrator(turtle)

    while not turtle.is_shutting_down():
        # test frequency
        # get point cloud



        i += 1;
        if i == period:
            ## registration
            # get checkpoints

            time_zero_reg = time.time()
            odom = turtle.get_odometry()
            pc = turtle.get_point_cloud()  # this takes approx 1/4 seconds

            dp_image, checkpoints, markers, ids = markreg.get_checkpoints(turtle.get_rgb_image(),
                                                                          turtle.get_depth_image(), pc, reg)
            dp_image = np.int8(dp_image / 7.0 * 255)
            dp_image = cv2.applyColorMap(255 - dp_image.astype(np.uint8), cv2.COLORMAP_JET)
            cv2.imshow(WINDOW, dp_image)
            reg.add_checkpoints(checkpoints, markers, ids, odom)


            reg.register_pc(pc, odom)
            time_reg = time.time()
            print('registered in ' + str(time_reg - time_zero_reg))
            # print(str(time.time()-zero)+' sec to compute bird view')
            # print(np.unique(image[:,:,0]))
            bird_view_image = reg.get_colored_map(odom)
            cv2.imshow(WINDOW_bird, bird_view_image)
            time_plot = time.time()
            print('plot in ' + str(time_plot - time_reg))


            ## profiling
            i = 0
            time_one = time.time()
            print(str(time_one - time_prev) + ' elapsed in ' + str(period) + ' cycles')
            time_prev = time_one
            print('we are at ' + str(odom[0:2]) + ', bearing ' + str(odom[2]))
            print('we are active = ' + str(active) + ' and going to ' + state_str(state))


            # show image
            cv2.waitKey(1)

        # command velocity
        if active and state == MOVE:
            turtle.cmd_velocity(linear=linear_vel)
            direction = None
        elif active and state == ROTATE:
            if direction is None:
                direction = np.sign(np.random.rand() - 0.5)
            turtle.cmd_velocity(angular=direction * angular_vel)
        elif active and state == STOP:
            turtle.cmd_velocity(linear=0, angular=0)
            direction = None


if __name__ == "__main__":
    main()
