import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial as spa
import time
from rrt_utils import *
from map_registrator import *


class GlobalPlanner(object):
    """Plan path between two points using RRT"""

    def __init__(self, threshold):
        self.GOAL_DISTANCE_THRESHOLD = threshold

        self.XDIM = [-3, 3]  # allowed dimensions for planning
        self.YDIM = [-3, 3]

    def samplePositionAtRandom(self):  # delete probably at all
        # # sample the space by a random point
        # # it is sligthly slower, because the samples often fall to the yard and the potential steps point into walls
        rand = Node(np.random.random() * (self.XDIM[1] - self.XDIM[0]) + self.XDIM[0],
                    np.random.random() * (self.YDIM[1] - self.YDIM[0]) + self.YDIM[0])
        # slightly more informed sampling - only in the space of the building, not in the yard area
        # there are two areas, the A area is cca 95m2  and the B area 216m2, so 1/3 times goes A, the rest B
        # to get approximately uniform sampling in the given space
        return rand

    def plan(self, start, goal, P):

        NUMNODES = 3000  # maximum number of nodes of a tree

        kd_tree = spa.cKDTree(P)  # KD tree used to check for collision points
        P=P.T
        #zero = time.time()

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
            randPos = self.samplePositionAtRandom()
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
            #print('kdtreed in '+str(first-zeroth) + ' for ' + str(len(nodes)) + ' nodes')
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
            else:
                continue

            # if ||newPos-goal||<GOAL_DISTANCE_THRESHOLD then terminate
            if np.linalg.norm(newPos.pos-goal.pos) < self.GOAL_DISTANCE_THRESHOLD:
                print 'found the path from ', newPos.pos.T, ' to ',goal.pos.T, ' in ', np.linalg.norm(newPos.pos-goal.pos)
                break
            second=time.time()
            #print('collided in ' + str(second-first))
            collided.append(second-first)
            if len(collided)==300:
                print np.array(collided).mean()
                print np.array(balled_).mean()
                print np.array(inlied_).mean()
                #break

        print 'WE ARE ', len(nodes), " STRONG"
        if verbose:
            print "final pos_list"
            print pos_list

        plotSolution(P, nodes, goal)

        return nodes[-1]
