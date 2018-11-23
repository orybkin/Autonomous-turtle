
import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from src.utils.path_controller import *

from src.utils.rrt_utils import *
from src.utils.rrt_planner import *
from src.utils.map_registrator import *

reg = MapRegistrator()
path_controller=PathController(None,reg)

np.set_printoptions(2)

ROT = path_controller.ROTATE
MOV = path_controller.MOVE
trajectory = [(MOV, 1), (ROT, 90), (MOV, 2), (ROT, 180), (MOV, 1), (MOV, 1), (ROT, 270), (MOV, 1)]
trajectory = [(ROT, 5),(MOV, 1), (ROT, 45), (MOV, 2), (ROT, 180), (MOV, 1), (MOV, 1), (ROT, 180), (MOV, 1)]
path_controller.set_commands(trajectory, [0,0,0])
path= path_controller.path
print path

path_controller=PathController(None,reg)

path_controller.set_path(path, [0,0,0])
print zip(path_controller.actions,path_controller.values)

if False:
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
    start = Node(1.75, -1.5)  # Start in the corner, in front of our office    # CHANGE
    goal = Node(2.75, 1)  # easy goal node used to check if finished       # CHANGE

    planner=GlobalPlanner()

    planner.plan(start,goal,P)
