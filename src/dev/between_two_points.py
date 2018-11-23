#!/usr/bin/env python

from b3m33aro import Turtlebot
import numpy as np
import time
import cv2
from math import *

MOVE = 1
ROTATE = 2
STOP = 3

linear_vel = 0.2
angular_vel = 0.5

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

    start_odom = turtle.get_odometry()

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
        if active and state == MOVE:
            go = 0.5 # odometry before turning
            if np.power((odom[0]-start_odom[0]),2) + np.power((odom[1]-start_odom[1]),2) > np.power(go,2):
                state = ROTATE
                start_odom = turtle.get_odometry()
            else:
                turtle.cmd_velocity(linear=linear_vel)
            direction = None

        if active and state == ROTATE:
            angle = 172.875  # desired angle to rotate the robot in degrees
            angle_rad = angle * np.pi /180  # desired angle in radians
            direction = 1  #1 to left rotation,  -1 is for opposite side?
            #if direction is None:
            print 'start angle ' + str(start_odom[2])
            print 'actual angle ' + str(odom[2])
            if odom[2] - start_odom[2] > angle_rad:
                goal_odom = turtle.get_odometry()
                state = STOP
                direction = None
            else:
                turtle.cmd_velocity(angular=direction * angular_vel)


        if active and state == STOP:
            state = MOVE
            start_odom = turtle.get_odometry()
 #           go = 0.5
 #           if np.power((odom[0]-goal_odom[0]),2) + np.power((odom[1]-goal_odom[1]),2) > np.power(go,2):
 #               start_odom = turtle.get_odometry()  # odometry before turning
 #               state = ROTATE

            #if for 90 degrees
 #           if (range_4[0]<start_odom[2]<range_4[1] and range_1[0]<odom[2]<range_1[1] and odom[2]\
 #                   + np.abs(start_odom[2])>angle_rad) or \
 #                   (range_2[0]<start_odom[2]<range_2[1] and range_3[0]<odom[2]<range_3[1] and \
 #                            odom[2] -(np.pi - start_odom[2])>angle_rad) or\
 #                            (odom[2] - start_odom[2] > angle_rad):

if __name__ == "__main__":
    main()
