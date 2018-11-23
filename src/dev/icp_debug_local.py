import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import time

def showPointClouds(P,Q,idxp, idxq,downsample=100):
    plt.cla()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(P[0, ::downsample], P[1, ::downsample], P[2, ::downsample], c='r', s=5, marker='.')
    ax.scatter(Q[0, ::downsample], Q[1, ::downsample], Q[2, ::downsample], c='k', s=5, marker='.')

load=1
# Ini
if load==1:
    P0 = np.load('../data/ICP/P0.npy')
    Q0 = np.load('../data/ICP/Q0.npy')
    Ks = np.r_[200*np.ones(100),1500*np.ones(10)].astype(int)
    xi=0.8
if load==2:
    P0 = np.load('../data/ICP/P0_large.npy')
    Q0 = np.load('../data/ICP/Q0_large.npy')
    Ks = np.r_[200*np.ones(20),2000*np.ones(10),20000*np.ones(5)].astype(int)
    xi=1
if load==3:
    P0 = np.load('../data/ICP/P0_large_secA.npy')
    Q0 = np.load('../data/ICP/Q0_large_secB.npy')
    Ks = np.r_[300*np.ones(10),3000*np.ones(10),30000*np.ones(5)].astype(int)
    xi=1
P = P0.copy()
Q = Q0.copy()
N = P.shape[1]
Kdswitch=10000

fig = plt.figure()
showPointClouds(P,Q, np.array([]), np.array([]))

#  grow a tree
if N>Kdswitch:
    zeroth=time.time()
    Q0Tree = KDTree(Q0.T)
    first=time.time()
    print "built tree in " + str(first-zeroth) + "sec"

##### ICP loop ######
for t in range(0,Ks.shape[0]):
    # Select random subset of points from P
    K=Ks[t]
    idxp = np.random.choice(N, K, False)

    # Find nearest neighbours in Q
    zeroth = time.time()
    if N>Kdswitch:
        dummy,idxq=Q0Tree.query(P[:,idxp].T)
        first = time.time()
        print "Kdtreed in " + str(first-zeroth) + "sec"
    else:
        d = np.zeros([K, N])
        for i in range(0, K):
            for j in range(0, P.shape[0]):
                d[i, :] = d[i, :] + np.power(P[j, idxp[i]] - Q0[j, :], 2)
        idxq = np.argmin(d, axis=1)
        first = time.time()
        print "NNed in " + str(first-zeroth) + "sec"

    idxp2=np.linalg.norm(P[:, idxp] - Q0[:, idxq],axis=0).argsort()
    idxp2=idxp2[:int(K*xi)]
    idxp=idxp[idxp2]
    idxq=idxq[idxp2]
    print('Step %d. Global Euclidean distance before: %.4f' % (t,np.linalg.norm(np.linalg.norm(P[:, idxp] - Q0[:, idxq], axis=0))))

    # Shift mean of the selected points to zero
    mP = np.mean(P[:, idxp], 1, keepdims=True)
    mQ = np.mean(Q0[:, idxq], 1, keepdims=True)
    P = P - np.tile(mP, [1, N])
    Q = Q0 - np.tile(mQ, [1, N])


    # Estimate rotation and translation
    H = P[:, idxp].dot(Q[:, idxq].T)
    U, s, V = np.linalg.svd(H, full_matrices=True)
    R = V.T.dot(U.T)
    T = mQ - R.dot(mP)

    # Transform pointcloud P to R*P+T
    P = R.dot(P + np.tile(mP, [1, N])) + np.tile(T, [1, N])

    # Compute alignment error
    print('Step %d. Global Euclidean distance after: %.4f' % (t,np.linalg.norm(np.linalg.norm(P[:, idxp] - Q0[:, idxq], axis=0))))

    plt.pause(0.02)
    # Show pointclouds
    subs=1
    showPointClouds(P[:,::subs], Q0[:,::subs], np.array([]), np.array([]),1)
    #showPointClouds(P[:,::(N/100)], Q0[:,::(N/100)], idxp, idxq)

showPointClouds(P, Q0, np.array([]), np.array([]))
#######################
plt.show()
