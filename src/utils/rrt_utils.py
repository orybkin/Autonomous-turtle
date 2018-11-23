import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial as spa
import time

balled_=[]
inlied_=[]

print_utils=True

R = 0.27  # ball radius

EPSILON = 0.5   # step by which the ball can move in space


# Class representing a node
class Node:
    pos = np.array([[0, 0]]).T
    parent = None

    def __init__(self, x, y=None):
        if y is None:
            self.pos = x.T
        else:
            self.pos = np.array([[x, y]]).T

    def to_path(self):
        node=self
        path=np.zeros((2,0))
        while not node.parent is None:
            path=np.c_[node.pos, path]
            print node.pos
            plt.plot([node.parent.pos[0, 0], node.pos[0, 0]],
                     [node.parent.pos[1, 0], node.pos[1, 0]], color='g')
            node = node.parent
        return path.T

# Plots points of a pointcloud P onto axes ax
def showPointClouds(P,ax):
    downsample=1
    ax.scatter(P[0, ::downsample], -P[1, ::downsample], c='r', s=5, marker='.')

# Draws the tree graph stored in nodes list
def plotNodes(nodes,plt):
    # plot the whole tree
    # for node in reversed(nodes):
    #     if node.parent is None:
    #         continue;
    #     plt.plot([node.parent.pos[0,0], node.pos[0,0]],
    #               [node.parent.pos[1,0], node.pos[1,0]], color='b')
    # plot the winning trajectory
    node = nodes[-1]
    while not node.parent is None:
        plt.plot([node.parent.pos[0,0], node.pos[0,0]],
                  [-node.parent.pos[1,0], -node.pos[1,0]], color='g')
        node = node.parent

# This function counts points that lay between the starting and final position of a ball
def points_inside_trajectory(P, x1, x2, r, P_KD_tree=None):
    if P_KD_tree is None:
        P_KD_tree = spa.cKDTree(P.T)
        print 'ACHTUNG'

    if print_utils:
        zero=time.time()
    # half of the x1 - x2 distance
    r2 = np.linalg.norm((x1 - x2)) * 0.5

    # points inside the big sphere
    #P_inliers = P_KD_tree.query_ball_point(((x1 + x2) * 0.5).T, r2 + r)[0]
    P_inliers = P_KD_tree.query_ball_point(x2.T, r)[0]
    P_inliers[len(P_inliers):]  = P_KD_tree.query_ball_point(((x1 + x2) * 0.5).T, r2)[0]
    P_inliers_filt = []

    if print_utils:
        balled=time.time()
        balled_.append(balled-zero)
        #print('balled in '+str(balled-zero))
    # points inside the tube
    for i in P_inliers:
        if np.linalg.norm(np.cross((P[:, [i]] - x1).T, (P[:, [i]] - x2).T)) / np.linalg.norm(x2 - x1) < r:
            P_inliers_filt.append(i)

    P_inliers = list(P_inliers_filt)
    P_inliers_filt = []
    P_inliers_potential = []

    # points inside the smaller sphere
    small_radius = np.sqrt(r2 ** 2 + r ** 2)

    for i in P_inliers:
        if np.linalg.norm((x1 + x2) / 2 - P[:, [i]]) < small_radius:
            P_inliers_filt.append(i)
        else:
            P_inliers_potential.append(i)

    # points lying inside the smallest radii around x1 or x2
    for i in P_inliers_potential:
        if np.minimum(np.linalg.norm(P[:, [i]] - x1), np.linalg.norm(P[:, [i]] - x2)) < r:
            P_inliers_filt.append(i)
    if print_utils:
        inlied = time.time()
        inlied_.append(inlied-balled)
        #print('inlied in '+str(inlied-balled))
    # sort the inliers and return
    P_inliers_filt.sort()
    return P_inliers_filt

# Counts collision points after calling points inside trajectory and returns true of false
def isCollision(parent_node, pos, P_KD_Tree, P, R, COLLISION_THRESH=6):
    collisions = points_inside_trajectory(P, parent_node.pos, pos, R, P_KD_Tree)
    return judge_collisions(collisions, COLLISION_THRESH)

def judge_collisions(collisions, COLLISION_THRESH):
    if len(collisions) > COLLISION_THRESH:
        return True
    else:
        return False

def plotSolution(P, nodes,goal):
    fig=plt.figure(1)
    plt.axis('equal')
    plt.cla()
    plotNodes(nodes, plt)

    plt.scatter(goal.pos[0, 0], goal.pos[1, 0], s=50, color='g')
    showPointClouds(P, plt)
    #plt.draw()
    plt.pause(0.01)
    fig.show()

