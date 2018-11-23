#!/usr/bin/env python


# Script for analysis of the turtlebot behaviour:
# find out frequency of the main cycle
# find out odometry measure
# check odometry and SfM correctness
# check the script correctness (optional)

from b3m33aro import Turtlebot
import numpy as np
import cv2
import time
from math import *
from utils.map_registrator import MapRegistrator

MOVE = 1
ROTATE = 2
STOP = 3
linear_vel = 0.2
angular_vel = 0.5
# linear_vel = 1
# angular_vel = 1
WINDOW = 'obstacles'
WINDOW1 = 'birdview'
WINDOW2 = 'rgb'
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
    turtle = Turtlebot(rgb=SHOW_RGB, pc=True)
    direction = None

    cv2.namedWindow(WINDOW)
    cv2.namedWindow(WINDOW1)
    cv2.setMouseCallback(WINDOW, click)
    cv2.moveWindow(WINDOW, 30, 30)
    cv2.moveWindow(WINDOW1, 30, 450)
    if SHOW_RGB:
        cv2.namedWindow(WINDOW2)

    # used for testing
    time_prev = time_zero = time.time()
    i = 0
    period = 10;
    turtle.reset_odometry()
    time.sleep(1)
    reg = MapRegistrator()
    while not turtle.is_shutting_down():
        # test frequency
        # get point cloud
        odom = turtle.get_odometry()
        pc = turtle.get_point_cloud()  # this takes approx 1/4 seconds
        if pc is None:
            print('NO POINT CLOUD DETECTED')
            continue

        if SHOW_RGB and turtle.get_rgb_image() is None:
            print('NO IMAGE DETECTED')
            continue

        # get image from points
        def get_im_color(mask):
            # empty image
            image = np.zeros(mask.shape)
            # assign depth i.e. distance to image
            image[mask] = np.int8(pc[:, :, 2][mask] / 3.0 * 255)
            im_color = cv2.applyColorMap(255 - image.astype(np.uint8), cv2.COLORMAP_JET)
            return im_color

        # masking out nan
        mask = np.ones(pc[:, :, 0].shape) == 1
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 0]))
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 1]))
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 2]))

        cv2.imshow(WINDOW, get_im_color(mask))


        if SHOW_RGB:
            cv2.imshow(WINDOW2, turtle.get_rgb_image())

        if np.count_nonzero(mask) <= 0:
            print('HOUSTON, WE HAVE A PROBLEM')
            continue

        i += 1;
        if i == period:
            i = 0
            time_one = time.time()
            print(str(time_one - time_prev) + ' elapsed in ' + str(period) + ' cycles')
            time_prev = time_one
            print('we are at ' + str(odom[0:2]) + ', bearing ' + str(odom[2]))
            print('we are active = ' + str(active) + ' and going to ' + state_str(state))
            print(pc[:, :, 0][mask].max(), pc[:, :, 0][mask].min())
            print(pc[:, :, 1][mask].max(), pc[:, :, 1][mask].min())
            print(pc[:, :, 2][mask].max(), pc[:, :, 2][mask].min())

            reg.register_pc(pc, odom)
            image = reg.get_colored_map(odom)
            # print(str(time.time()-zero)+' sec to compute bird view')
            # print(np.unique(image[:,:,0]))
            cv2.imshow(WINDOW1, image)
        mseconds = int(floor(10 * (time.time() - time_zero)))
        # np.save("pc_"+str(mseconds), pc)
        # np.save("odom_"+str(mseconds), odom)
        print('saving ' + str(odom[2]))

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
