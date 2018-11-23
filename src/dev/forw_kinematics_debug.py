
import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from src.utils.path_controller import *



ROT = 1
MOV = 2
commands = [(MOV, 1), (ROT, 90), (MOV, 2), (ROT, 180), (MOV, 1), (MOV, 1), (ROT, 270), (MOV, 1)]

actions, values= zip(*commands)

rad2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
deg2rad = lambda a : a * np.pi/180
deg2R = lambda a : rad2R(deg2rad(a))

# robot position and orientation in projective coordinates
robot=np.array([[0, 0]]).T

# forward kinematics for all joints
path=np.array([[], []])
for command in reversed(commands):
    action=command[0]
    value=command[1]
    if action==MOV:
        print robot.shape
        print path.shape
        path=np.c_[robot, path]
        path[0]=path[0]+value
    if action==ROT:
        path=deg2R(value).dot(path)

print path.T