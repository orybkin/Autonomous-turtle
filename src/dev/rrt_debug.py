import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial as spa
import time
from src.utils.rrt_utils import *
from src.utils.map_registrator import *


NUMNODES = 3000  # maximum number of nodes of a tree

P = np.load('../../data/sample/pc_321.npy')# point cloud map   # CHANGE
reg = MapRegistrator()
zero=time.time()
print('KDtree build in '+str(time.time()-zero))
print reg.pc_to_bird(P).shape
reg.register_pc(P,[0, 0, 0])
reg.compress_points()
P=reg.points.T
print P.shape

kd_tree = spa.cKDTree(P.T)  # KD tree used to check for collision points

# main function
start = Node(1.75,-1.5)       # Start in the corner, in front of our office    # CHANGE
goal = Node(2.75,1)      # easy goal node used to check if finished       # CHANGE

# Initialize search tree
nodes = []
nodes.append(start)

pos_list = np.array([[],[]])
pos_list = np.hstack([pos_list,start.pos])
collided=[]
verbose=False

# While we have not reached max num of nodes
while len(nodes) <= NUMNODES:

    zeroth=time.time()
    # sample position at random
    randPos = samplePositionAtRandom()
    pos_tree = spa.cKDTree(pos_list.T)
    dist, idx = pos_tree.query((randPos.pos).T)
    if verbose:
        print "pos_list is:"
        print pos_list
        print "KDTree of nodes positions after " + str(len(nodes)) + " nodes containing is:"
        print pos_tree.data
        print "distance is " + str(dist)
        print "index is " + str(idx)

    # find the closest node to randPos
    for node in nodes:
        if node.pos[0][0] == pos_list[:, idx][0] and node.pos[1][0] == pos_list[:, idx][1]:
            nNode = node
            break
    first=time.time()
    print('kdtreed in '+str(first-zeroth) + ' for ' + str(len(nodes)) + ' nodes')
    # estimate new position (if ||randPos-nNode||<EPSILON, newPos=randPos otherwise shorten the step to EPSILON
    if dist < EPSILON:
        newPos = randPos
    else:
        xyzn=(nNode.pos+(randPos.pos-nNode.pos)*EPSILON / dist).T[0]
        newPos = Node(xyzn[0], xyzn[1])
        # pos = np.array([[0, 0]]).T
        # print xn[0]
        # print yn[0]
        # print zn[0]
    newPos.parent = nNode
    if verbose:
        print "parent is:"
        print (newPos.parent).pos
        print "estimated position is:"
        print newPos.pos

    # if collision-free path from nNode to newPos exists then add newPos to the tree
    # syntax: isCollision(nNode, newPos, kd_tree, P, R)
    if isCollision(nNode, newPos.pos, kd_tree, P, R) == False:
        if verbose:
            print "no collision -> node is added to nodes"
        nodes.append(newPos)
        pos_list = np.hstack([pos_list, newPos.pos])

    # if ||newPos-goal||<GOAL_DISTANCE_THRESHOLD then terminate
    if np.linalg.norm(newPos.pos-goal.pos) < GOAL_DISTANCE_THRESHOLD:
        break
    second=time.time()
    print('collided in ' + str(second-first))
    collided.append(second-first)
    if len(collided)==300:
        print np.array(collided).mean()
        print np.array(balled_).mean()
        print np.array(inlied_).mean()
        #break

if verbose:
    print "final pos_list"
    print pos_list

plotSolution(P, nodes)
plt.show()