#!/usr/bin/env python

from b3m33aro import Turtlebot
import numpy as np
import time
import cv2
from math import *

MOVE = 1
ROTATE = 2
STOP = 3

linear_vel = 0.5
angular_vel = 0.5

range_1 = (0, np.pi /2)
range_2 = (np.pi / 2, np.pi)
range_3 = (-np.pi, -np.pi/ 2)
range_4 = (-np.pi / 2, 0)

WINDOW = 'between two points'

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

    time_zero=time.time()
    i=0
    period=2000;

    #actions_list =
    #values_list =

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
            print ('start odometry is ' + str(start_odom[0:3]))
            print ('angles difference is ' + str(odom[2] - start_odom[2]))
        odom = turtle.get_odometry()

        #robot, go for 2 meters and STOP
        if active and state == MOVE:
            go = 0.5 # odometry before turning
            if np.power((odom[0]-start_odom[0]),2) + np.power((odom[1]-start_odom[1]),2) > np.power(go,2):
                state = ROTATE
                start_odom = turtle.get_odometry()
                print ('move is done, start odometry is ' + str(start_odom[0:2]))
            else:
                turtle.cmd_velocity(linear=linear_vel, angular=0)
            direction = None

        if active and state == ROTATE:
            goal = 90  # desired angle to rotate the robot in degrees #172.875
            deviation=8
            goal_rad = goal * np.pi /180
            deviation_rad = deviation * np.pi /180  # desired angle in radians
            direction = 1  #1 to left rotation,  -1 is for opposite side?
            #if direction is None:
            #print 'start angle ' + str(start_odom[2])
            #print 'actual angle ' + str(odom[2])

            # angle through which we already rotated
            angle_reached= odom[2] - start_odom[2]
            if angle_reached<0:
                angle_reached=angle_reached+2*pi
            if angle_reached>=2*pi:
                angle_reached=angle_reached-2*pi

            # difference between angle through which we already rotated and goal angle
            angle_diff=angle_reached-goal_rad
            if angle_diff<-pi:
                angle_diff=angle_diff+2*pi
            if angle_diff>=pi:
                angle_diff=angle_diff-2*pi

            # command
            if abs(angle_diff) < deviation_rad:
                start_odom = turtle.get_odometry()
                state = MOVE
                direction = None
                print ('rotation is done, start odometry is ' + str(start_odom[0:2]))
            else:
                turtle.cmd_velocity(angular=direction * angular_vel, linear=0)

if __name__ == "__main__":
    main()
