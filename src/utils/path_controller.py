#!/usr/bin/env python



import numpy as np
import cv2
import time
from math import *
from icp import icp
from PID import *


class PathController(object):
    """Low-level robot path control"""

    MOVE = 1
    ROTATE = 2
    STOP = 3
    DONE = 4 # same as stop

    COMMANDS=1 # mode of the robot. whether we are following commands or path
    PATH=2

    def __init__(self, turtle, reg):
        self.actions = np.array([[]])
        self.values = np.array([[]])
        self.action = self.STOP  # idle

        self.turtle = turtle
        self.reg=reg
        self.time_prev_comm = time.time() # time we executed previous command
        self.time_prev_check = time.time() # time we executed previous check

        deg2rad = lambda a: a * np.pi / 180
        self.deviation = deg2rad(2) # tolerable deviation for angle
        self.linear_vel = 0.5
        self.angular_vel = 0.5
        self.direction = None

        self.angle_PID=PID()

    def set_commands(self, commands, odometry=None):
        """Set new trajectory and start movement immediately. REMEMBER NOT TO END IN A ROTATION"""
        actions, values= zip(*commands)

        deg2rad = lambda a: a * np.pi / 180
        self.actions = np.array(actions)
        self.values = np.array(values)

        if odometry is None: odometry=self.turtle.get_odometry()

        ## forward kinematics for all joints. compute path
        angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
        robot=np.zeros((1,2))
        path=np.zeros((0,2))

        for command in reversed(commands):
            action=command[0]
            value=command[1]
            if action==self.MOVE:
                print robot.shape, path.shape
                path=np.r_[robot, path]
                path[:,0]=path[:,0]+value
            if action==self.ROTATE:
                path=path.dot(angle2R(deg2rad(value)))
        self.path=self.reg.current2standard(odometry, path)

        self.mode=self.COMMANDS

        self.restart(odometry)

    def set_path(self, path, odom):
        self.path=path
        print 'odometry while setting path ', odom
        print path
        path=self.reg.standard2current(odom, path)
        print path

        actions=[]
        values=[]

        for i in range(path.shape[0]):
            actions.append(self.ROTATE)
            # here comes trigonometry
            if path[i,0]>0:
                # add SoS
                if path[i,0]==0:
                    angle=pi/2
                else:
                    angle=atan(path[i,1]/path[i,0])
            else:
                if path[i,0]==0:
                    angle=-pi/2 # the limit
                else:
                    angle=atan(-path[i,1]/path[i,0])
                angle=pi-angle
            #print (path[i])
            #print angle
            rad2deg = lambda a: a * 180 / pi
            values.append(rad2deg(-angle))
            actions.append(self.MOVE)
            values.append(np.linalg.norm(path[i]))

            # move _path_ to current joint coordinate system
            angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
            path_curr = path - path[i]
            path_curr = path_curr.dot(angle2R(angle))
            path=path_curr

        self.actions=actions
        self.values=values

        self.mode=self.PATH

        self.restart(odom)

    def restart(self, odometry):
        # restart the robot
        if self.path.shape[0]==0:
            self.action=self.DONE
            print 'DONE'
            return
        if odometry is None: odometry=self.turtle.get_odometry()
        self.iterator_action = 0
        self.iterator_goal = 0
        self.start_odom = odometry # odometry when starting current goal

        self.goal=self.get_current_goal()
        self.start_action()

    def next_action(self, odom):
        if self.get_current_action()==self.MOVE:
            self.iterator_goal += 1

            if self.iterator_goal >= self.path.shape[0]:
                self.action = self.DONE
                return

            self.goal=self.get_current_goal()

        self.iterator_action += 1
        print self.iterator_action, len(self.actions)
        print self.iterator_goal, self.path.shape

        self.start_action()
        odom1=self.turtle.get_odometry()
        time.sleep(0.1)
        #print ('odometry change after sleeping is ' + str(odom1-self.turtle.get_odometry()))
        self.start_odom = odom1

    def start_action(self):
        self.goal_value = self.get_current_value()
        self.action = self.get_current_action()
        if self.action==self.MOVE and self.goal_value>0.05:
            self.go_fast()
        else:
            self.go_normal()



    def get_current_action(self):
        return self.actions[self.iterator_action]

    def get_current_value(self):
        return self.values[self.iterator_action]

    def get_current_goal(self):
        return self.path[self.iterator_goal]


    def check(self):
        if self.action == self.MOVE:
            self.turtle.cmd_velocity(angular=0)

        if self.action == self.ROTATE:
            # positive velocity rotates left
            self.turtle.cmd_velocity(linear=0)

    def normalize_angle(self, angle, bottom, top=0):
        if angle < bottom:
            angle = angle + 2 * pi
        if angle >= (bottom + 2 * pi):
            angle = angle - 2 * pi
        return angle


    def frame_update(self, odom=None):
        """This method should be called at least at 100 Hz for normal functioning"""

        if odom is None: odom=self.turtle.get_odometry()

        deg2rad = lambda a: a * np.pi / 180

        if self.action == self.MOVE:

            angle_diff = self.normalize_angle(odom[2] - self.start_odom[2], -pi, pi)
            angular_PID = self.angle_PID.frame_update(angle_diff)

            if (odom[0] - self.start_odom[0]) ** 2 + (odom[1] - self.start_odom[1]) ** 2 > self.goal_value ** 2:
                print ('MOVE DONE, moved '  + str(self.start_odom[2] - odom[2])
                       + ', deviation in m ' + str(np.linalg.norm(self.start_odom[0:2]-odom[0:2]))+
                       ' deviation in rad ' + str(self.start_odom[2] - odom[2]))
                print self.goal_value

                self.turtle.cmd_velocity(0,0)
                print 'we are ' + str(np.linalg.norm(self.reg.odom2bird_coordinates(odom)-self.goal)) + ' m off track'
                print 'we are in the commands mode = ', self.mode==self.COMMANDS
                if self.mode==self.COMMANDS:
                    self.next_action(odom)
                else:
                    self.set_path(self.path[1:],odom)
            else:
                self.turtle.cmd_velocity(linear=self.linear_vel, angular=-angular_PID)


        if self.action == self.ROTATE:
            # convert to radians

            # angle through which we already rotated
            angle_reached = self.normalize_angle(odom[2] - self.start_odom[2], 0, 2 * pi)

            # difference between angle through which we already rotated and goal angle
            angle_diff = self.normalize_angle(angle_reached - deg2rad(self.goal_value), -pi, pi)


            if self.direction is None:
                print 'chosing direction, angle diff: ', angle_diff
                if angle_diff<0:
                    self.direction=1
                else:
                    self.direction=-1

            # command
            if abs(angle_diff) < self.deviation:
                print ('ROTATION DONE, rotated ' + str(self.start_odom[2] - odom[2])
                       + ', deviation in m ' + str(np.linalg.norm(self.start_odom[0:2]-odom[0:2]))+
                       ' deviation in rad ' + str(angle_diff))
                print self.goal_value
                self.turtle.cmd_velocity(0,0)
                self.next_action(odom)
                self.direction=None
            else:
                self.turtle.cmd_velocity(angular=self.direction * self.angular_vel, linear=0)


    def go_fast(self):
        self.angular_vel=1
        self.linear_vel=1

    def go_normal(self):
        self.angular_vel=1
        self.linear_vel=0.5
    def go_slow(self):

        self.angular_vel=0.2
        self.linear_vel=0.2

    def pause(self):
        self.turtle.cmd_velocity(linear=0, angular=0)

