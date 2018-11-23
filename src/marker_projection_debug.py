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
    turtle = Turtlebot(rgb=True, pc=True, depth=True)
    turtle.reset_odometry()
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
    reg = MapRegistrator()
    markreg = MarkerRegistrator(turtle)
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

        ## get depth image
        def get_im_color(mask):
            # empty image
            image = np.zeros(mask.shape)
            # assign depth i.e. distance to image
            image[mask] = np.int8(pc[:, :, 2][mask] / 7.0 * 255)
            im_color = cv2.applyColorMap(255 - image.astype(np.uint8), cv2.COLORMAP_JET)
            return im_color

        # masking out nan
        mask = np.ones(pc[:, :, 0].shape) == 1
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 0]))
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 1]))
        mask = np.logical_and(mask, ~np.isnan(pc[:, :, 2]))
        ## get markers
        markers = detector.detect_markers(turtle.get_rgb_image())
        print (' number of markers is ' + str(len(markers)))
        dp_image = turtle.get_depth_image()
        dp_image1 = dp_image.copy()
        if len(markers) > 0:
            ids = np.array([d[1] for d in markers])
            dets = [np.array(d[0]) for d in markers]
            print ids, dets
            ## get bird view
            image = reg.pc_to_bird(pc)
            fig1 = plt.figure()
            plt.axis('equal')
            plt.scatter(image[:, 0], -image[:, 1], c='g', s=1)
            colors = ['r', 'g', 'b']
            for i in range(len(dets)):
                if ids[i] == 0:
                    print 'noise'
                    continue
                det = dets[i]
                print dp_image.shape
                if det.min()<20 or (det.max(axis=0)>([dp_image.shape[1]-20, dp_image.shape[0]-20])).any():
                    print det.min(), det.max(axis=0)
                    print 'edge'
                    continue
                checkpoint = markreg.get_3d_marker(dp_image1, det)
                checkpoint, marker_pc = markreg.get_3d_marker_pc(pc, det)
                if checkpoint is None:
                    print 'not estimated'
                    continue
                checkpoint = reg.pc2bird_coordinates(checkpoint)
                marker_pc = reg.pc2bird_coordinates(marker_pc)
                plt.scatter(checkpoint[:, 0], -checkpoint[:, 1], c=colors[i], s=50)
                plt.scatter(marker_pc[:, 0], -marker_pc[:, 1], c=colors[i], s=50)
            plt.show()
        dp_image = dp_image1
        dp_image = np.int8(dp_image / 7.0 * 255)
        dp_image = cv2.applyColorMap(255 - dp_image.astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow(WINDOW, dp_image)
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