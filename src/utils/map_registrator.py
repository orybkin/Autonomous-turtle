#!/usr/bin/env python


# Script for analysis of the turtlebot behaviour:
# find out frequency of the main cycle
# find out odometry measure
# check odometry and SfM correctness
# check the script correctness (optional)

import numpy as np
import cv2
import time
from math import *
from icp import icp
import scipy.spatial as spa


class MapRegistrator(object):
    """serves for constructing a map of robot environment (in 2d bird view space)"""

    # SUPPOSED RESOLUTION OF A FRAME POINT CLOUD
    TOP_Y_MIN = -2.0
    TOP_Y_MAX = 2.0
    TOP_Z_MIN = -4.0
    TOP_Z_MAX = -0.0
    TOP_X_MIN = 0.5
    TOP_X_MAX = 4

    # MAP RESOLUTION
    SCALE = 0.02

    OBSTACLE = 1
    FREE = 0
    CHECKPOINT = 2
    ROBOT = 3

    def __init__(self):
        self.points = np.array([[]])
        self.previous = None
        self.checkpoints = dict()
        self.checkpoints_visited = dict()

    def pc_to_bird(self, pcloud):
        """convert (3d) point cloud to (2d) bird view coordinates"""

        if len(pcloud.shape) > 2:
            pcloud = pcloud[5:-5, 5:-5, :]
            pcloud = pcloud.reshape((-1, 3))
        # maybe  print minmax
        '''
        print(pxs.max(),pxs.min())
        print(pys.max(),pys.min())
        print(pzs.max(),pzs.min())
        '''
        # not nan
        pcloud = pcloud[~np.isnan(pcloud[:, 0])]
        pcloud = pcloud[~np.isnan(pcloud[:, 1])]
        pcloud = pcloud[~np.isnan(pcloud[:, 2])]

        # crop to remove clutter
        pcloud = pcloud[pcloud[:, 0] < self.TOP_Y_MAX]
        pcloud = pcloud[pcloud[:, 0] > self.TOP_Y_MIN]
        pcloud = pcloud[pcloud[:, 1] < self.TOP_Z_MAX]
        pcloud = pcloud[pcloud[:, 1] > self.TOP_Z_MIN]
        pcloud = pcloud[pcloud[:, 2] < self.TOP_X_MAX]
        pcloud = pcloud[pcloud[:, 2] > self.TOP_X_MIN]

        return self.pc2bird_coordinates(pcloud)

    def odom2bird_coordinates(self, odometry):
        return np.array([[odometry[0], -odometry[1]]])


    def pc2bird_coordinates(self, pcloud):
        # transform coordinates
        pxs = pcloud[:, 2]  # perednost'
        pys = pcloud[:, 0]  # tudamsudamnost'
        pzs = pcloud[:, 1]  # height
        return np.array([pxs, pys]).transpose()

    def depth2bird_coordinates(self, pcloud):
        # transform coordinates
        pxs = pcloud[:, 2]
        pys = pcloud[:, 1]
        pzs = pcloud[:, 0]
        return np.array([pxs, pys]).transpose()

    def add_checkpoints(self, checkpoints, markers, ids, odom):
        for i in range(len(ids)):
            checkpoint = self.current2standard(odom, self.pc2bird_coordinates(checkpoints[i]))
            marker_pc = self.current2standard(odom, self.pc2bird_coordinates(markers[i]))
            self.checkpoints[ids[i]] = checkpoint
            if not(ids[i] in self.checkpoints_visited):
                self.checkpoints_visited[ids[i]] = False

    def get_unvisited(self, odom):
        odom = self.odom2bird_coordinates(odom)
        min = 10
        imin=None;
        for i in self.checkpoints:
            if not self.checkpoints_visited[i]:
                distance = np.linalg.norm(odom - self.checkpoints[i])
                if distance< min:
                    imin=i

        if imin is None:
            return None, None
        else:
            return imin,self.checkpoints[imin]


    # def get_unvisited(self, odom):
    #     for i in self.checkpoints:
    #         if not self.checkpoints_visited[i]:
    #             return i,self.checkpoints[i]
    #     return None, None

    def mark_visited(self,id):
        self.checkpoints_visited[id]=True

    def register_pc(self, pcloud, odometry=None):
        """add point cloud to map"""
        points = self.pc_to_bird(pcloud)
        if odometry is None:
            points = icp(pcloud, self.previous)
            print('ALARM')
        else:
            points = self.current2standard(odometry, points)

        if self.points.shape[1] == 0:
            self.points = points
        else:
            # sparsify
            # time_z=time.time()
            # tree1=spa.cKDTree(self.points)
            # tree2 = spa.cKDTree(points)
            # print(self.points.shape,points.shape)
            # neighbours=tree1.query_ball_tree(tree2,0.1)
            # far=neighbours
            # print('treed')
            # for i in range(len(neighbours)):
            #     far[i]= len(neighbours[i])==0
            # print('cycled')
            # self.points=np.r_[self.points,points[far,:]]
            # print('KDTreed in '+str(time.time()-time_z))
            #
            # print 'points in the map: ' + str(self.points.shape)

            self.points = np.r_[self.points, points]
            self.compress_points()

    def reset_odometry(self, odometry):
        """change the zero point of the map"""
        self.points = self.current2standard(odometry, self.points)

    def current2standard(self, odometry, points):
        """move points captured in _odometry_ coordinates to standard coordinates"""
        angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
        points = points.dot(angle2R(odometry[2]))
        points = points + self.odom2bird_coordinates(odometry)
        return points

    def standard2current(self, odometry, points):
        """move points captured in standard coordinates to odometry coordinates"""
        angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
        points = points - self.odom2bird_coordinates(odometry)
        points = points.dot(angle2R(-odometry[2]))
        return points

    def get_map(self, odom):
        """get a binary map of the environment"""

        # 2d positions
        if len(self.checkpoints)>0:
            checkpoints=np.concatenate(self.checkpoints.values())
        else:
            checkpoints=np.zeros((0,2))
        #print checkpoints.shape, self.points.shape, self.odom_to_bird_coordinates(odom).shape
        points = np.r_[self.points, self.odom2bird_coordinates(odom), checkpoints]

        # object labels
        obstacles = self.OBSTACLE * np.ones(self.points.shape[0])
        checkpoints = self.CHECKPOINT * np.ones(len(self.checkpoints))
        objects = np.r_[obstacles, self.ROBOT, checkpoints]

        map, dummy, dummy1, dummy2 = self.discretize(points, objects)
        return map

    def clear_map(self):
        self.points = np.array([[]])
        self.previous = None

    def compress_points(self):
        map, zero, x, y=self.discretize(self.points)
        indices=map.nonzero()
        points=np.array([x[indices], y[indices]]).T
        # indices=np.array(map.nonzero()).T
        # points=indices*self.SCALE+zero + np.array([0.5, 0.5])*self.SCALE # dont create points in the centers of the rectangles
        self.points=points

    def discretize(self, points, objects=None):
        """creates a discretized image of the point, coloring them according to information in 'objects'"""

        if objects is None:
            objects=np.ones(points.shape[0])

        # it is possible to add height and reflection information to the map if needed
        pxs = points[:, 0]
        pys = points[:, 1]
        x_min = pxs.min()
        x_max = pxs.max()
        x_max = x_max + abs(x_max * 0.01)
        y_min = pys.min()
        y_max = pys.max()
        y_max = y_max + abs(y_max * 0.01)

        def find_highest_and_density_in_cell(pxs, pys):
            zero = time.time()
            idx_max = np.zeros(resolution, dtype=int)
            density = np.zeros(resolution)
            x = np.zeros(resolution)
            y = np.zeros(resolution)
            #print density.shape
            # preprocessed=time.time()
            # print('preprocessed in '+str(preprocessed-zero))

            cellx = np.asarray(np.floor((pxs - x_min) / self.SCALE), int)
            celly = np.asarray(np.floor((pys - y_min) / self.SCALE), int)

            density[cellx, celly] = objects
            x[cellx,celly]=pxs
            y[cellx,celly]=pys
            height_sort = np.arange(N)
            idx_max[cellx[height_sort], celly[height_sort]] = height_sort

            return idx_max, density, x, y

        N = pxs.size

        # map resolution
        x_resolution = (x_max - x_min) / self.SCALE
        y_resolution = (y_max - y_min) / self.SCALE
        resolution = (int(ceil(x_resolution)), int(ceil(y_resolution)))
        #print(resolution)
        #print(x_max - x_min / self.SCALE)

        # preprocessed=time.time()
        # print('preprocessed in '+str(preprocessed-zero))

        idx_max, density, x,y = find_highest_and_density_in_cell(pxs, pys)


        return density, [x_min, y_min], x, y

    def get_colored_map(self,odom):
        map=self.get_map(odom)

        # pump up the robot
        robot = (map==self.ROBOT).nonzero()
        #print ("ROBOT pumpin")
        self.pump_up(map,(robot[0][0],robot[1][0]),3)
        # pump up the checkpoints
        checks = (map==self.CHECKPOINT).nonzero()
        #print ("OBST pumpin")
        for i in range(len(checks[0])):
            self.pump_up(map,(checks[0][i],checks[1][i]),3)

        #print 'unique map ' + str(np.unique(map))
        #print 'unique map ' + str(np.unique(np.flipud((map[:, :] *50).astype(np.uint8))))
        image = cv2.applyColorMap(np.flipud((map[:, :] *60).astype(np.uint8)), cv2.COLORMAP_JET)

        np.save('bird_image.npy',image)

        #print image[robot[0],robot[1],:]

        return image

    def pump_up(self, map, object, pix):
        #print object
        #print (object[0] - pix)
        #print map.shape
        map[(object[0] - pix) : (object[0] + pix), (object[1] - pix) : (object[1] + pix)]
        map[object]
        map[(object[0] - pix) : (object[0] + pix), (object[1] - pix) : (object[1] + pix)]=map[object]