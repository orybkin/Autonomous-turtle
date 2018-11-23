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
linear_vel = 0.5
angular_vel = 0.5
#linear_vel = 1
#angular_vel = 1
WINDOW = 'depth map'
WINDOW1 = 'point cloud '
running = True
active = True

def state_str(state):
    if state==STOP:
        return("STOP")
    elif state==ROTATE:
        return("ROTATE")
    elif state == MOVE:
        return ("MOVE")

def click(vent, x, y, flags, param):
    if vent!=1:
        return
    global state
    global direction
    if y<100:
        state=MOVE
    elif y<260:
        state=ROTATE
        if x>240:
            direction=-1
        else:
            direction=+1
    else:
        state=STOP
    print('coordinates ' + str(x) + ' ' + str(y))
    print('robot is going to ' + state_str(state))



def main():
    global state
    global direction
    state=STOP
    turtle = Turtlebot(pc=True,depth=True)
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
            print('we are active = ' + str(active) + ' and going to '+ state_str(state))
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

        # depth image
        # depth=turtle.get_depth_image()
        #
        # mask = ~np.isnan(depth)
        # image = np.zeros(mask.shape)
        # image[mask] = np.int8(depth[:, :][mask] / 3.0 * 255)
        # im_color = cv2.applyColorMap(255 - image.astype(np.uint8), cv2.COLORMAP_JET)
        #
        # cv2.imshow(WINDOW1, im_color)
        # cv2.waitKey(1)
        # command velocity
        if active and state == MOVE:
            turtle.cmd_velocity(linear=linear_vel)
            direction = None
        elif active and state == ROTATE:
            if direction is None:
                direction = np.sign(np.random.rand() - 0.5)
            turtle.cmd_velocity(angular=direction * angular_vel)
        elif active and state == STOP:
            turtle.cmd_velocity(linear=0,angular=0)
            direction = None


if __name__ == "__main__":
    main()
