
import numpy as np
import cv2
import time
from math import *
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from b3m33aro import Turtlebot, detector


class MarkerRegistrator(object):


    def __init__(self,turtle):
        print turtle.rgb_camera_info
        print turtle.depth_camera_info

        self.P_rgb = turtle.rgb_camera_info.P
        self.P_dp = turtle.depth_camera_info.P

        self.P_rgb = np.array(self.P_rgb).reshape((3, 4))
        self.P_dp = np.array(self.P_dp).reshape((3, 4))
        self.P_dp_inv = pinv(self.P_dp)
        self.P_rgb_inv = pinv(self.P_rgb)
        print(self.P_rgb, self.P_dp)

        self.K_rgb = turtle.rgb_camera_info.K
        self.K_dp = turtle.depth_camera_info.K
        self.K_rgb = np.array(self.K_rgb).reshape((3, 3))
        self.K_dp = np.array(self.K_dp).reshape((3, 3))
        self.K_dp_inv = pinv(self.K_dp)

    def get_3d_marker(self, dp_image, marker):

        hull_0 = cv2.convexHull(marker)
        #print hull_0
        points_3D_list = []
        points_2D_list = []

        for x in range(0, dp_image.shape[0],5):
            for y in range(0, dp_image.shape[1],5):
                if ~np.isnan(dp_image[x, y]):
                    # for each point in depth image
                    x_rgb=self.K_rgb.dot(self.K_dp_inv.dot([[x], [y], [1]]))

                    # for some reason markers have (y,x) coordinate system
                    if cv2.pointPolygonTest(hull_0, (x_rgb[1], x_rgb[0]), False)>0:
                        x_world=(dp_image[x, y] * self.P_rgb_inv.dot(x_rgb))
                        x_redp=self.P_dp.dot(x_world)
                        #print x_redp/x_redp[-1],x,y
                        points_3D_list.append(x_world)
                        dp_image[x,y]=6

        print len(points_3D_list)
        if len(points_3D_list)==0:
            return None
        points_3D = np.concatenate(points_3D_list, axis=1)
        #print(points_3D[:,0:5])
        med = np.median(points_3D, axis=1)
        print points_3D.shape
        return points_3D.T
        #return np.array([med[0:3]])

    def get_3d_marker_pc(self, pc, marker):

        hull_0 = cv2.convexHull(marker)
        #print hull_0
        points_3D_list = []
        points_2D_list = []

        if pc.shape[0]==480:
            subsample=10
        else:
            subsample=5

        for x in range(0, pc.shape[0],subsample):
            for y in range(0, pc.shape[1],subsample):
                if all(~np.isnan(pc[x, y])):
                    # for each point in depth image
                    x_rgb=self.K_rgb.dot(self.K_dp_inv.dot([[x], [y], [1]]))

                    # for some reason markers have (y,x) coordinate system
                    if cv2.pointPolygonTest(hull_0, (x_rgb[1], x_rgb[0]), False)>0:
                        x_world=pc[x, y]
                        x_redp=self.P_dp[0:3, 0:3].dot(x_world)
                        #print x_redp/x_redp[-1],x,y
                        points_3D_list.append(np.array([x_world]).T)

        print len(points_3D_list)
        #print(points_3D_list)
        if len(points_3D_list)==0:
            return None, None
        points_3D = np.concatenate(points_3D_list, axis=1)
        #print(points_3D[:,0:5])
        med = np.median(points_3D, axis=1)
        checkpoint=self.get_checkpoint(points_3D)
        # Plot
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # subsam = 1
        # ax.scatter(points_3D[:, 0], points_3D[:, 1], points_3D[:, 2], s=10)
        # ax.scatter(checkpoint[:,0], checkpoint[:,1], checkpoint[:,2], c='r', s=10)
        #
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        #
        # plt.show()
        # end plot
        print points_3D.shape
        return checkpoint, points_3D.T
        return
        #return np.array([med[0:3]])

    def get_checkpoint(self,marker):
        # hand-filter the outliers?
        z=np.median(marker,axis=1)[2]

        # fit the plane
        marker=marker.T
        center=np.mean(marker,axis=0)
        marker=marker-center
        u,s,v=np.linalg.svd(marker)
        normal=v[2,:]

        # uprighting
        up = np.array([0, 0, 1]) # vector facing up
        # normal=normal-up * up.dot(normal) # project normal down
        # normal=normal/np.linalg.norm(normal)
        normal=normal*np.sign(normal[2]) # normal is directed away from us

        checkpoint=center-normal*0.7
        return np.array([checkpoint])

    def get_checkpoints(self,rgb_image,dp_image,pc,reg ):
        markers = detector.detect_markers(rgb_image)
        dp_image1 = dp_image.copy()
        checkpoints=[]
        markers_points=[]
        idxs=[]
        if len(markers) > 0:
            ids = np.array([d[1] for d in markers])
            dets = [np.array(d[0]) for d in markers]
            print ids, dets
            ## get bird view
            image = reg.pc_to_bird(pc)
            #fig1 = plt.figure()
            #plt.axis('equal')
            #plt.scatter(image[:, 0], -image[:, 1], c='g', s=1)
            #colors = ['r', 'g', 'b']
            for i in range(len(dets)):
                if ids[i] == 0:
                    # spurious marker - noise
                    #print 'noise'
                    continue
                det = dets[i]

                if dp_image.shape[0] == 480:
                    edge = 70
                else:
                    edge = 20

                if det.min()<edge or (det.max(axis=0)>([dp_image.shape[1]-edge, dp_image.shape[0]-edge])).any():
                    #print 'too close to edge'
                    continue
                #checkpoint = self.get_3d_marker(dp_image1, det)
                checkpoint, marker_pc = self.get_3d_marker_pc(pc, det)
                if checkpoint is None:
                    #print 'cannot be estimated'
                    continue
                checkpoints.append(checkpoint)
                markers_points.append(marker_pc)
                idxs.append(ids[i])
                #plt.scatter(checkpoint[:, 0], -checkpoint[:, 1], c=colors[i], s=50)
                #plt.scatter(marker_pc[:, 0], -marker_pc[:, 1], c=colors[i], s=50)
            #plt.show()

        print (' number of markers is ' + str(len(markers_points)))
        return dp_image1, checkpoints, markers_points, idxs