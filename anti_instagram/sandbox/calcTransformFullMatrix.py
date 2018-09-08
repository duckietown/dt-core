#!/usr/bin/env python

import numpy as np
from anti_instagram.kmeans_rebuild import *
import cv2
from outlierEstimation_trafo2 import *
from plotHelper import *

"""
This class calculates the optimal transform parameters based on the following description.
We want to map the input color space [B, G, R] to the 'true' color space [B', G', R'].

The color transform:     |a_B    b_B    c_B|       |B|     |B'|
                         |a_G    b_G    c_G|   *   |G| =   |G'|
                         |a_R    b_R    c_R|       |R|     |R'|


Since we have 9 unknown parameters we need 3 centers to calculate the parameters.



The inputs:
- found centers:          this is an array containing the 3 centers of the clusters.
- true centers:           this is the array containing the 3 true centers

The outputs:
- transformationMatrix    this is the transformation matrix containing the 9 parameters
"""

class calcTransform:


    true_centers = []          # 3 x 3 array, containing n 'true' centers in BGR
    num_centers = -1           # 3
    found_centers = []         # 3 x 3 array, containing n found centers in BGR
    valueArrayBGR = []
    transformationMatrix = []  # 3 x 3 array
    true_centers_matrix = []   # 3 x 3 array, containing the 'true' colors for each channel
    trained_centers_matrix = []  # 3 x 3 array, containing the 'trained' colors for each channel
    residuals = []
    residualNorm = -1

    # initialize
    def __init__(self, numOcenters, found_centers, true_centers = []):
        assert (numOcenters >= 2), "at least two centers needed. Otherwise under constrained"
        self.num_centers = numOcenters
        # if no true centers are provided
        self.true_centers = true_centers
        self.found_centers = found_centers
        self.scale = np.zeros(3, np.float64)
        self.shift = np.zeros(3, np.float64)
        self.residuals = np.zeros((3, 3), np.float64)
        self.valueArrayBGR = np.zeros((3, self.num_centers), np.uint8)
        self.true_centers_transposed = np.matrix.transpose(self.true_centers)
        self.trained_centers_transposed = np.matrix.transpose(self.found_centers)

        print('created instance of calcTransform!')

    def returnResidualNorm(self):
        return self.residualNorm

    def calcTransform(self):
        print "the trained centers: " + str(self.found_centers)
        print "the trained centers transposed: " + str(self.trained_centers_transposed)
        print "the true centers: " + str(self.true_centers)
        print "the true centers transposed: " + str(self.true_centers_transposed)
        invertedTrained = np.linalg.pinv(self.trained_centers_transposed)
        self.transformationMatrix = np.dot(self.true_centers_transposed, invertedTrained)

    def applyTransform(self, img):
        print "trafo matrix: " + str(self.transformationMatrix)
        image_transformed = np.dot(img, self.transformationMatrix)
        return image_transformed

def main():
    KM = kMeansClass(20, 'median', 0.1, 5)
    image = cv2.imread("test_images/pic3.jpg", cv2.IMREAD_UNCHANGED)
    KM.applyKM(image)
    idxBlack, idxRed, idxYellow, idxWhite = KM.determineColor(KM.trained_centers)
    trained_centers = np.array([KM.trained_centers[idxBlack], KM.trained_centers[idxRed],
                                KM.trained_centers[idxYellow], KM.trained_centers[idxWhite]])
    print "the trained centers: " + str(trained_centers)
    plotKMeansCenters(KM.trained_centers, KM.labels)
    plotDeterminedCenters(trained_centers[0], trained_centers[2], trained_centers[3], trained_centers[1])
    print("plot is done.")
    true_centers = np.vstack([[70, 50, 60], [50, 70, 240], [60, 240, 230], [250, 250, 250]])
    '''
    outlierIndex, outlierCenter = detectOutlier(trained_centers, true_centers)
    true_centers_woOutlier = np.delete(true_centers, outlierIndex, 0)
    trained_centers_woOutlier = np.delete(trained_centers, outlierIndex, 0)
    '''

    T = calcTransform(3, trained_centers[0:2], true_centers[0:2])
    T.calcTransform()
    image_transformed = T.applyTransform(image)

    print "shape of the transformed image: " + str(image_transformed.shape)
    print "the trained center transformed [black]: " + str(T.applyTransform(trained_centers[0]))
    print "the trained center transformed [red]: " + str(T.applyTransform(trained_centers[1]))

    cv2.namedWindow('input', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('input', image)
    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', image_transformed)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    sys.exit()
