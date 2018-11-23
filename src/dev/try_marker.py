#!/usr/bin/env python

from b3m33aro import Turtlebot, detector
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from src.utils.map_registrator import MapRegistrator
from src.utils.marker_registrator import MarkerRegistrator


MOVE = 1
ROTATE = 2
STOP = 3

WINDOW = 'rgb'
WINDOW1 = 'depth with marked '

points_3D_list = []
x_3D = []
y_3D = []
z_3D = []

def main():
    turtle = Turtlebot(rgb=True,depth=True)
    # get rgb camera info

    cv2.namedWindow(WINDOW)
    cv2.namedWindow(WINDOW1)

    markreg=MarkerRegistrator(turtle)

    while not turtle.is_shutting_down():


        # get point cloud
        image = turtle.get_rgb_image()

        # wait for image to be ready
        if image is None:
            continue

        # detect markers in the image
        markers = detector.detect_markers(image)
        print (' number of markers is ' + str(len(markers)))
        if len(markers) > 0:
            ids = np.array([d[1] for d in markers])
            dets = [np.array(d[0]) for d in markers]
            print(ids, dets)
        else:
            continue

        # draw markers in the image
        detector.draw_markers(image, markers)

        # show image
        cv2.imshow(WINDOW, image)
        cv2.waitKey(1)

        # marker corners coordinates, i for all markers, for now just for one marker
        # for i in len(dets):
        # dets[i][0]

        #x1 = dets[0][0, 0]
        #y1 = dets[0][0, 1]
        #x2 = dets[0][1, 0]
        #y2 = dets[0][1, 1]
        #x3 = dets[0][2, 0]
        #y3 = dets[0][2, 1]
        #x4 = dets[0][3, 0]
        #y4 = dets[0][3, 1]

        print(dets[0])
        dp_image = turtle.get_depth_image()
        print(dp_image.shape)
        dp_image1=dp_image.copy()

        #print ('depth is ' + str(dist))

        checkpoint = markreg.get_3d_marker(dp_image1, dets[0])

        if len(checkpoint)==0:
            continue
        dp_image=dp_image1
        dp_image = np.int8(dp_image / 7.0 * 255)
        dp_image = cv2.applyColorMap(255 - dp_image.astype(np.uint8), cv2.COLORMAP_JET)
        detector.draw_markers(dp_image, markers)
        cv2.imshow(WINDOW1, dp_image)
        cv2.waitKey(1)

        #print points_3D
        print ('median of 1st c. is ' + str(checkpoint[0]))
        print ('median of 2st c. is ' + str(checkpoint[1]))
        print ('median of 3st c. is ' + str(checkpoint[2]))
        time.sleep(1)
        #print points_3D_list
        #x_3D = points_3D_list[0]
        #y_3D = points_3D_list[1]
        #z_3D = points_3D_list[2]
        #med_x = np.median(x_3D)
        #med_y = np.median(y_3D)
        #med_z = np.median(z_3D)

if __name__ == "__main__":
    main()
