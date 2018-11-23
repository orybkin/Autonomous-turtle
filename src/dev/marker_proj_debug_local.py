#!/usr/bin/env python


import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


X=np.random.rand(3,200)
normal=np.array([[1, 1, 1]])
normal=normal/np.linalg.norm(normal)
X=X-normal.T.dot(normal).dot(X)
X=X.T
center=np.mean(X,axis=0)
X=X-center
u,s,v=np.linalg.svd(X)
normal=v[2,:]

up = np.array([0, 0, 1]) # vector facing up
normal=normal-up * up.dot(normal) # project normal down
normal=normal/np.linalg.norm(normal)
normal=normal*np.sign(normal[1]) # normal is directed away from us

checkpoint=center-normal*1.5

fig=plt.figure()
ax = fig.add_subplot(111, projection='3d')
subsam=1
ax.scatter(X[:,0],X[:,1],X[:,2],s=10)
ax.scatter(checkpoint[0],checkpoint[1],checkpoint[2],c='r',s=10)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.quiver(0,0,0, normal[0],normal[1],normal[2] )

plt.show()

print v