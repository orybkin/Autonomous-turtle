import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import cKDTree
import time
from math import *


def register(P1,Q1):
    # Shift mean of the selected points to zero
    N=P1.shape[1]
    mP = np.mean(P1, 1, keepdims=True)
    mQ = np.mean(Q1, 1, keepdims=True)
    P1 = P1 - np.tile(mP, [1, N])
    Q1 = Q1 - np.tile(mQ, [1, N])

    if P1.shape[0] == 3:
        # Estimate rotation and translation
        H = P1.dot(Q1.T)
        U, s, V = np.linalg.svd(H, full_matrices=True)
        R = V.T.dot(U.T)  # there possibly should be V instead of V.T
        if np.linalg.det(R) < 0:
            raise Exception('determinant is negative. see http://nghiaho.com/?page_id=671')

        # Transform pointcloud P to R*P+T
        #P = R.dot(P1) + np.tile(mQ, [1, N])
        t=R.dot(-mP)+mQ
    else:
        angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
        l = atan((P1[0, :] * Q1[1, :] - P1[1, :] * Q1[0, :]).sum() / (P1[0, :] * Q1[0, :] + P1[1, :] * Q1[1, :]).sum())
        R = angle2R(l)

        # P = R.dot(P1) + np.tile(mQ, [1, N])
        t=R.dot(-mP)+mQ
    return R,t

def icp(P0,Q0):
    P = P0.copy()

    Np = P.shape[1]
    Nq = Q0.shape[1]
    N=min(Np,Nq)
    Kdswitch=1000
    Ks = np.r_[100 * np.ones(10), 1000 * np.ones(10), 10000 * np.ones(5)].astype(int)

    #  grow a tree
    if N>Kdswitch:
        zeroth=time.time()
        Q0Tree = cKDTree(Q0.T)
        first=time.time()
        print "built tree in " + str(first-zeroth) + "sec"

    fig = plt.figure()

    fig.show()
    ##### ICP loop ######
    for it in range(0,Ks.shape[0]):
        # Select random subset of points from P

        #K=Ks[t]
        K=N
        idxp = np.random.choice(N, K, False)

        # Find nearest neighbours in Q
        zeroth = time.time()
        if N>Kdswitch:
            dummy,idxq=Q0Tree.query(P[:,idxp].T)
            first = time.time()
            print "Kdtreed in " + str(first-zeroth) + "sec"
        else:
            d = np.zeros([K, Nq])
            for i in range(0, K):
                #print i
                #print idxp[i]
                for j in range(0, Q0.shape[0]):
                    #print j
                    d[i, :] = d[i, :] + np.power(P[j, idxp[i]] - Q0[j, :], 2)
            idxq = np.argmin(d, axis=1)
            first = time.time()
            print "NNed in " + str(first-zeroth) + "sec"

        idxp2=np.linalg.norm(P[:, idxp] - Q0[:, idxq],axis=0).argsort()
        #idxp2=idxp2[:int(idxp2.shape[0]*xi)]
        idxp=idxp[idxp2]
        idxq=idxq[idxp2]

        print('Step %d. Global Euclidean distance before: %.4f' % (it,np.linalg.norm(np.linalg.norm(P[:, idxp] - Q0[:, idxq], axis=0))))

        R,t = register(P[:, idxp], Q0[:, idxq])
        P = R.dot(P) + np.tile(t, [1, Np])

        # Compute alignment error
        print('Step %d. Global Euclidean distance after: %.4f' % (it, np.linalg.norm(np.linalg.norm(P[:, idxp] - Q0[:, idxq], axis=0))))

        def showPointClouds(P, Q, idxp, idxq, downsample=100):
            plt.cla()
            plt.scatter(P[0, ::downsample], P[1, ::downsample], c='r', s=5, marker='.')
            plt.scatter(Q[0, ::downsample], Q[1, ::downsample], c='k', s=5, marker='.')

        subs = 1
        showPointClouds(P[:, ::subs], Q0[:, ::subs], np.array([]), np.array([]), 1)

    plt.show()
    return P