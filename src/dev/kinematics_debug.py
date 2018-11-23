
import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from spyder.utils.sourcecode import path_components

from src.utils.path_controller import *

from src.utils.rrt_utils import *
from src.utils.rrt_planner import *
from src.utils.map_registrator import *

reg = MapRegistrator()
path_controller=PathController(None,reg)

np.set_printoptions(2)

ROT = path_controller.ROTATE
MOV = path_controller.MOVE
trajectory = [(ROT, -80),(MOV, 1), (ROT, 200), (MOV, 2)]
trajectory = [(ROT, 0),(MOV, 1), (ROT, 45), (MOV, 2), (ROT, 180),
              (MOV, 1), (MOV, 1), (ROT, 180), (MOV, 1),(ROT, 90), (MOV, 1)]
trajectory = [(MOV, 1), (ROT, 90), (MOV, 2), (ROT, 180), (MOV, 1), (MOV, 1), (ROT, 270), (MOV, 1)]
odom=[4,4,-pi/2]
odom=[0,0,0]
path_controller.set_commands(trajectory, odom)
path= path_controller.path
print path

path_controller=PathController(None,reg)

path_controller.set_path([[1, 0]], [1,0,pi/2])
path_controller.set_path(path, odom)
print zip(path_controller.actions,path_controller.values)

if True:
    # plt.figure()
    # plt.axis('equal')
    # plt.scatter(path_controller.path[:,0],path_controller.path[:,1], color='g')
    # plt.show()
    pass
else:
    P = np.load('../../data/sample/pc_321.npy')  # point cloud map   # CHANGE
    reg = MapRegistrator()
    #zero = time.time()
    #print('KDtree build in ' + str(time.time() - zero))
    print reg.pc_to_bird(P).shape
    reg.register_pc(P, [0, 0, 0])
    reg.compress_points()
    P = reg.points
    print P.shape

    # main function
    start = Node(0, 0)  # Start in the corner, in front of our office    # CHANGE
    goal = Node(2.75, 1)  # easy goal node used to check if finished       # CHANGE
    goal = Node(0.5, 0.5)  # easy goal node used to check if finished       # CHANGE

    planner=GlobalPlanner()
    path_nodes=planner.plan(start, goal, P)
    #print path_nodes.to_path()
    path_controller.set_path(path_nodes.to_path(), [0,0,0])
    print path_controller.path
    print zip(path_controller.actions, path_controller.values)
    plt.scatter(path_controller.path[:,0],path_controller.path[:,1], color='b')
    plt.show()

