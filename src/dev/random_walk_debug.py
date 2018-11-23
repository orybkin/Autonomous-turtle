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

MOVE = 1
ROTATE = 2
STOP = 3
linear_vel = 0.2
angular_vel = 0.3
WINDOW = 'obstacles'
running = True
active = True


def click(vent, x, y, flags, param):
    if vent!=1:
        return
    global active
    active = not active
    print('robot is active = ' + str(active))



def main():
    turtle = Turtlebot(pc=True)
    direction = None
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, click)
    turtle.reset_odometry()
    # used for testing
    time_zero=time.time()
    i=0
    period=10;
    while not turtle.is_shutting_down():
        # test frequency
        i+=1;
        if i==period:
            i=0
            time_one=time.time()
            print(str(time_one-time_zero) + ' elapsed in ' + str(period) + ' cycles')
            time_zero=time_one
            odom=turtle.get_odometry()
            print('we are at ' + str(odom[0:2]) + ', bearing ' + str(odom[2]))
            print('OBSTACLE DEAD AHEAD. ROTATING')
        # get point cloud
        pc = turtle.get_point_cloud()
        if pc is None:
            print('NO POINT CLOUD DETECTED')
            continue
        # mask out floor points
        mask = pc[:, :, 1] < 0.2
        # mask point too far
        mask = np.logical_and(mask, pc[:, :, 2] < 3.0)
        if np.count_nonzero(mask) <= 0:
            print('HOUSTON, WE HAVE A PROBLEM')
            continue
        # empty image
        image = np.zeros(mask.shape)
        # assign depth i.e. distance to image
        image[mask] = np.int8(pc[:, :, 2][mask] / 3.0 * 255)
        im_color = cv2.applyColorMap(255 - image.astype(np.uint8), cv2.COLORMAP_JET)
        # show image
        cv2.imshow(WINDOW, im_color)
        cv2.waitKey(1)
        # check obstacle
        mask = np.logical_and(mask, pc[:, :, 1] > -0.2)
        data = np.sort(pc[:, :, 2][mask])
        state = MOVE
        if data.size > 50:
            dist = np.percentile(data, 10)
            if dist < 0.6:
                state = ROTATE
        # command velocity
        if active and state == MOVE:
            turtle.cmd_velocity(linear=linear_vel)
            direction = None
        elif active and state == ROTATE:
            if direction is None:
                direction = np.sign(np.random.rand() - 0.5)
            turtle.cmd_velocity(angular=direction * angular_vel)

if __name__ == "__main__":
    main()
