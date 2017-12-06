#!/usr/bin/env python
import numpy as np
from anti_instagram.kmeans import CENTERS, CENTERS2

"""
This class calculates the optimal transform parameters based on the linear least squares optimization.
(y = a*x + b)
I.e. it solves the problem b=Ax, where x are the transform parameters, B the desired color value and A the color value of the found centers.

Since there are 3 color channels, we got 6 unknown parameters: for each channel a scale and a shift parameter (a, b).
Thus we need ad least 3 cluster centers in order to get enough equations. For 4 cluster centers, we have to solve an over constrained sysOeq.

The inputs:
- number of centers:      this is the number of centers which are used to compute the transform
- found centers:          this is an array containing the centers of the clusters.

The outputs:
- scale and shift:        The scale and shift values correspond to 'a' and 'b' in y = a*x + b for each channel
"""

class calcTransform:

    # hardcoded centers from anti_instagram.kmeans
    centers_BYW = CENTERS
    centers_BRYW = CENTERS2

    centers = []
    num_centers = -1
    found_centers = []
    valueArrayBGR = []
    valueArrayGreen = []
    valueArrayRed = []
    matrices_A = []
    matrices_B = []
    scale = []
    shift = []

    # initialize
    def __init__(self, numOcenters, found):
        assert (numOcenters >= 2), "at least two centers needed. Otherwise under constrained"
        self.num_centers = numOcenters
        if self.num_centers == 4:
            self.centers = self.centers_BRYW
        elif self.num_centers == 3:
            self.centers = self.centers_BYW

        self.found_centers = found
        self.scale = np.zeros(3, np.float64)
        self.shift = np.zeros(3, np.float64)
        self.valueArrayBGR = np.zeros((3, self.num_centers), np.uint8)
        for k in range(3): self.valueArrayBGR[k,:] = self.found_centers[:,k]

        self.matrices_A = np.zeros((3, self.num_centers, 2), np.uint8)
        self.matrices_B = np.zeros((3, self.num_centers), np.uint8)
        for channel in range(3):
            # prepare vectors b for each channel
            self.matrices_B[channel] = self.centers[:, channel]
            # prepare matrices A for each channel
            self.matrices_A[channel] = np.array(([[self.valueArrayBGR[channel, j], 1] for j in range(self.num_centers)]))
        print(self.matrices_A)

        print('created instance of calcTransform!')


    def calcTransform(self):
        # loop over the three color channels
        for channel in range(3):
            # calculate least squares
            X = np.linalg.lstsq(self.matrices_A[channel],self.matrices_B[channel])[0]
            self.scale[channel] = X[0]
            self.shift[channel] = X[1]
        print(self.scale)
        print(self.shift)


# colors in bgr
#[60,60,60] is dark grey
#[60, 60, 240] is red
#[50, 240, 240] is yellow
#[240, 240, 240] is white
# CENTERS2 = np.array([[60, 60, 60], [60, 60, 240], [50, 240, 240], [240, 240, 240]])
#found_centers = np.array([[50, 55, 50], [50, 65, 220], [55, 250, 230], [250, 240, 230]])
found_centers = np.array([[50, 55, 50], [55, 250, 230], [250, 240, 230]])

T = calcTransform(3, found_centers)
T.calcTransform()