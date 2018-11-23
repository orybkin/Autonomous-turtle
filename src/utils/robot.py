
import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from b3m33aro import Turtlebot, detector


from path_controller import *
from rrt_planner import *
from marker_registrator import *
from map_registrator import *



class Robot(object):

    CIRCLE=1
    PLAN_PATH=2
    GO=3

    GOAL_DISTANCE_THRESHOLD=0.2

    WINDOW_DEPTH='depth image'
    WINDOW_BIRD='bird view'
    WINDOW_RGB='rgb'

    def __init__(self,turtle):

        self.reg = MapRegistrator()
        self.markreg = MarkerRegistrator(turtle)
        self.path_controller = PathController(turtle, self.reg)
        self.turtle=turtle
        self.planner=GlobalPlanner(self.GOAL_DISTANCE_THRESHOLD)

        # some windows
        cv2.namedWindow(self.WINDOW_BIRD)
        cv2.namedWindow(self.WINDOW_DEPTH)
        #cv2.namedWindow(self.WINDOW_RGB)
        #cv2.setMouseCallback(self.WINDOW_DEPTH, click)
        cv2.moveWindow(self.WINDOW_DEPTH, 30, 30)
        cv2.moveWindow(self.WINDOW_BIRD, 600, 100)

        self.time_update=1

        self.time_prev_reg = time.time() # time we previously registrated points

    def register_obstacles(self, add_checkpoints=True):
        """Add currently seen obstacles to the map"""

        time.sleep(0.2)
        odom = self.turtle.get_odometry()
        pc = self.turtle.get_point_cloud()

        self.reg.register_pc(pc, odom)
        time_reg = time.time()
        # print(str(time.time()-zero)+' sec to compute bird view')
        # print(np.unique(image[:,:,0]))

        if add_checkpoints:
            #zero=time.time()
            self.add_checkpoints(odom,pc)
            #print ('registered checkpoints in ' + str(time.time()-zero))

        bird_view_image = self.reg.get_colored_map(odom)
        cv2.imshow(self.WINDOW_BIRD, bird_view_image)
        cv2.waitKey(1)
        # adding checkpoints to the map also doesn't hurt

    def add_checkpoints(self, odom=None, pc=None):
        """Add checkpoints to the map (currently computationally expensive)"""
        if odom is None:
            odom = self.turtle.get_odometry()
        if pc is None:
            pc = self.turtle.get_point_cloud()  # this takes approx 1/4 seconds

        dp_image, checkpoints, markers, ids = self.markreg.get_checkpoints(self.turtle.get_rgb_image(),
                                                                           self.turtle.get_depth_image(), pc, self.reg)
        dp_image = np.int8(dp_image / 7.0 * 255)
        dp_image = cv2.applyColorMap(255 - dp_image.astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow(self.WINDOW_DEPTH, dp_image)
        cv2.waitKey(1)
        self.reg.add_checkpoints(checkpoints, markers, ids, odom)

    def estimate_scene_dimensions(self):
        odom=self.turtle.get_odometry()
        print self.reg.checkpoints.values(), odom
        points = np.r_[np.concatenate(self.reg.checkpoints.values()), self.reg.odom2bird_coordinates(odom)]
        max=points.max(axis=1)
        min=points.min(axis=1)

        distance = 0.25

        self.planner.XDIM = [min[0] - distance, max[0] + distance]  # allowed dimensions for planning
        self.planner.YDIM = [min[1] - distance, max[1] + distance]

    def plan_path(self):
        """Plan path to one random checkpoint"""

        odom=self.turtle.get_odometry()
        reg=self.reg
        id,checkpoint=reg.get_unvisited(odom)
        if checkpoint is None:
            print "All visited"
            return None, None

        start = Node(reg.odom2bird_coordinates(odom))
        goal=Node(checkpoint)

        print 'marker #', id, ' chosen, start and end positions: ', start.pos.T,' ', goal.pos.T
        path_nodes = self.planner.plan(start, goal, reg.points)

        self.path_controller.set_path(path_nodes.to_path(), odom)
        print 'actions planned ', zip(self.path_controller.actions, self.path_controller.values), 'length ', len(
            self.path_controller.path)

        self.goal=checkpoint
        self.id=id

    def circle(self):
        ROT=self.path_controller.ROTATE
        MOV=self.path_controller.MOVE
        dir=np.sign(np.random.rand(1)-0.5) # circle direction
        commands=[(ROT, dir*90), (MOV, 0.01), (ROT, dir*90),(MOV, 0.01),
                  (ROT, dir*90),(MOV, 0.01), (ROT, dir*90), (MOV, 0.01)]
        self.path_controller.set_commands(commands,self.turtle.get_odometry())

    def look_around(self):
        ROT = self.path_controller.ROTATE
        MOV = self.path_controller.MOVE
        commands = [(ROT, -30), (MOV, 0), (ROT,  30), (MOV, 0), (ROT, 30), (MOV, 0)]
        self.path_controller.set_commands(commands, self.turtle.get_odometry())

        self.go_fast()

        while not self.turtle.is_shutting_down() and self.path_controller.action<>self.path_controller.DONE:
            self.frame_update(register_obstacles=True)

    def go_checkpoint(self):
        self.plan_path()

        if id is None:
            return

        # plt.figure()
        # plt.show()
        time.sleep(1)
        self.go_slow()
        self.time_prev_reg = time.time()

        while not self.turtle.is_shutting_down() and self.path_controller.action <> self.path_controller.DONE:
            self.frame_update(register_obstacles=False,replan=True)

        print 'goal.pos.T',self.goal
        self.mark_visited()

    def mark_visited(self, i=None):
        if i is not None:
            checkpoint = self.reg.checkpoints[i]
        else:
            checkpoint=self.goal
            i=self.id
        if np.linalg.norm(self.reg.odom2bird_coordinates(
                self.turtle.get_odometry()) - checkpoint) < self.GOAL_DISTANCE_THRESHOLD:

            self.turtle.play_sound()
            self.reg.mark_visited(i)
            print 'marker ', i, ' VISITED'

    def go_right_to_checkpoint(self):

        odom = self.turtle.get_odometry()
        reg = self.reg
        self.id, self.goal = reg.get_unvisited(odom)
        if self.goal is None:
            print "NO CHECKPOINTS FOUND"
            return


        self.path_controller.set_path(self.goal, odom)
        print 'actions planned ', zip(self.path_controller.actions, self.path_controller.values), 'length ', len(
            self.path_controller.path)

        self.go_slow()
        self.time_prev_reg = time.time()

        while not self.turtle.is_shutting_down() and self.path_controller.action <> self.path_controller.DONE:
            self.frame_update(register_obstacles=False)


        self.mark_visited()

        time.sleep(0.1)
        print self.turtle.get_odometry()

    def go_trajectory(self, commands, register_obstacles=False):
        self.go_slow()

        self.path_controller.set_commands(commands)

        while not self.turtle.is_shutting_down() and self.path_controller.action <> self.path_controller.DONE:
            self.frame_update(register_obstacles=register_obstacles)


    def frame_update(self, odom=None, register_obstacles=False,replan=False):
        #register_obstacles= True
        replan=False
        if odom is None: odom=self.turtle.get_odometry()

        if register_obstacles:
            if time.time() - self.time_prev_reg > 1:
                self.path_controller.pause()
                #zero=time.time()
                self.register_obstacles()
                if replan:
                    self.plan_path()
                self.time_prev_reg = time.time()
                #print ('registered all in ' + str(time.time()-zero))

        # can happen that we are going by a marker
        for i in self.reg.checkpoints:
            if not self.reg.checkpoints_visited[i]:
                self.mark_visited(i)

        self.path_controller.frame_update(odom)

    def go_fast(self):
        self.path_controller.angular_vel=1
        self.path_controller.linear_vel=1

    def go_normal(self):
        self.path_controller.angular_vel=0.5
        self.path_controller.linear_vel=0.5
    def go_slow(self):

        self.path_controller.angular_vel=0.5
        self.path_controller.linear_vel=0.2