#!/usr/bin/env python

from b3m33aro import Turtlebot
import numpy as np
import time
import cv2

MOVE = 1
ROTATE = 2
STOP = 3

linear_vel = 0.2
angular_vel = 0.3

WINDOW = 'try move'

running = True
active = True


def click(vent, x, y, flags, param):
    global active
    active = not active
    print active


def main():
    turtle = Turtlebot(pc=True)
    turtle.reset_odometry()
    direction = None

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, click)

    time_zero=time.time()
    i=0
    period=10;

    state = MOVE

    while not turtle.is_shutting_down():

        i += 1;
        if i == period:
            i = 0
            time_one = time.time()
            print(str(time_one - time_zero) + ' elapsed in ' + str(period) + ' cycles')
            time_zero = time_one
            print('we are at ' + str(odom[0:2]) + ', bearing ' + str(odom[2]))
        odom = turtle.get_odometry()

        #robot, go for 2 meters and STOP

        # command velocity
        if active and state == MOVE:
            go = 2
            if np.power(odom[0],2) + np.power(odom[1],2) > np.power(go,2):
                odom1 = turtle.get_odometry()  # odometry before turning
                state = ROTATE

            turtle.cmd_velocity(linear=linear_vel)
            direction = None

        if active and state == ROTATE:
            angle = 90 # desired angle to rotate the robot
            angle_rad = angle * np.pi /180 # desired angle in radians
            direction = 1 #1 to left rotation,  -1 is for opposite side?
            #if direction is None:
            turtle.cmd_velocity(angular=direction * angular_vel)
            odom2 = turtle.get_odometry()
            if (odom2[2] -odom1[2]) > angle_rad:
                state = STOP
                direction = None


if __name__ == "__main__":
    main()
