"""The file is deprecated"""

import numpy  as np
import time
import math

RESOLUTION = 448
X_SCALE=0
Y_SCALE=0

# supposed scene resolution
TOP_Y_MIN = -2.0
TOP_Y_MAX = 2.0
TOP_Z_MIN = -4.0
TOP_Z_MAX = -0.0
TOP_X_MIN = 0.5
TOP_X_MAX = 4


def pc_to_bird(pcloud, objects=None):
    global X_SCALE, Y_SCALE
    if len(pcloud.shape)>2:
        pcloud=pcloud.reshape((-1, 3))
    if False: # crop the field of view
        crop=20
        # TOP_X_MIN=-crop
        # TOP_X_MAX=crop
        # TOP_Y_MIN=-crop
        # TOP_Y_MAX=crop


    #
    # TOP_X_MAX=pcloud[:, 0].max()*1.01
    # TOP_X_MIN=pcloud[:, 0].min()
    # TOP_Z_MAX=pcloud[:, 1].max()*1.01
    # TOP_Z_MIN=pcloud[:, 1].min()
    # TOP_Y_MAX=pcloud[:, 2].max()*1.01
    # TOP_Y_MIN=pcloud[:, 2].min()

    if 1:
        # zero=time.time()
        pcloud = pcloud[pcloud[:, 0] < TOP_Y_MAX]
        pcloud = pcloud[pcloud[:, 0] > TOP_Y_MIN]
        pcloud = pcloud[pcloud[:, 1] < TOP_Z_MAX]
        pcloud = pcloud[pcloud[:, 1] > TOP_Z_MIN]
        pcloud = pcloud[pcloud[:, 2] < TOP_X_MAX]
        pcloud = pcloud[pcloud[:, 2] > TOP_X_MIN]

    # transform coordinates
    pxs = pcloud[:, 2]
    pys = pcloud[:, 0]
    pzs = pcloud[:, 1]



    N=pxs.size

    def find_highest_and_density_in_cell(pxs,pys,pzs,resolution):
        zero=time.time()
        idx_max=np.zeros((resolution,resolution),dtype=int)
        density=np.zeros((resolution,resolution))
        # preprocessed=time.time()
        # print('preprocessed in '+str(preprocessed-zero))

        cellx=np.asarray(np.floor((pxs-TOP_X_MIN)/X_SCALE),int)
        celly=np.asarray(np.floor((pys-TOP_Y_MIN)/Y_SCALE),int)

        density[cellx,celly]+=1
        #height_sort=pzs.argsort()
        height_sort=np.arange(pzs.size)
        idx_max[cellx[height_sort],celly[height_sort]]=height_sort

        # features_found=time.time()
        # print('features_found in '+str(features_found-preprocessed))
        # for i in range(0,N):
        #     cell=(math.floor((pxs[i]-TOP_X_MIN)/X_SCALE),math.floor((pys[i]-TOP_Y_MIN)/Y_SCALE))
        #     if pzs[i]>pzs[idx_max[cell]]:
        #         idx_max[cell]=i

        # postprocessed = time.time()
        # print('postprocessed in ' + str(postprocessed - features_found))

        return idx_max,density

    # pixels size
    X_SCALE= (TOP_X_MAX-TOP_X_MIN) / RESOLUTION
    Y_SCALE= (TOP_Y_MAX-TOP_Y_MIN) / RESOLUTION

    # preprocessed=time.time()
    # print('preprocessed in '+str(preprocessed-zero))

    # find features
    idx_max, density=find_highest_and_density_in_cell(pxs, pys, pzs, RESOLUTION)

    # features_found=time.time()
    # print('features_found in '+str(features_found-preprocessed))

    has_points=density!=0
    intensity=np.zeros((RESOLUTION, RESOLUTION))
    height=np.zeros((RESOLUTION, RESOLUTION))
    height[has_points]=pzs[idx_max][has_points]+TOP_Z_MIN

    #print(np.unique(density))

    image=np.array([density, height]).transpose([1,2,0])

    return image