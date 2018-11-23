#!/usr/bin/env python


import numpy as np
import cv2
import time
from math import *
from icp import icp


class PID(object):
    """PID controller for an abstarct variable"""

    # PID constants
    Ki = 0  # CHANGE
    Kd = 0  # CHANGE
    Kp = 0.5  # CHANGE

    def __init__(self):
        self.u = 0
        self.e = 0
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

    def frame_update(self, angle_diff):

        # Delta t
        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        # Integration Input
        up = self.Kp * angle_diff
        # Integration Input
        ui = self.u + self.Ki * dt * angle_diff
        # Derivation Input
        if dt > 0:  # in order not to divide by zero
            ud = self.Kd * (angle_diff - self.e) / dt
        # Adjust values
        self.e = angle_diff
        self.u = ui
        self.prevtm = self.currtm  # save t for next pass
        # Calculate input for the system - angle
        u = up + ui + ud
        return u