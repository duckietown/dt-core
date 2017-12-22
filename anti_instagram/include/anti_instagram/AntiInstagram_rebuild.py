#!/usr/bin/env python
from anti_instagram.kmeans_rebuild import *
from anti_instagram.calcLstsqTransform import *
from anti_instagram.simpleColorBalanceClass import *
from .scale_and_shift import scaleandshift
import numpy as np
import rospy


class ScaleAndShift():
    """ Represents the transformation """

    def __init__(self, scale, shift):
        self.scale = scale
        self.shift = shift

    def __call__(self, image):
        corrected_image = scaleandshift(image, self.scale, self.shift)
        return corrected_image

    @staticmethod
    def identity():
        return ScaleAndShift([1.0, 1.0, 1.0], [0.0, 0.0, 0.0])


class AntiInstagram():
    def __init__(self):
        # scale&shift for image trafo
        self.scale = [1.0, 1.0, 1.0]
        self.shift = [0.0, 0.0, 0.0]
        # thresholds for color balance
        self.ThLow = [0, 0, 0]
        self.ThHi = [255, 255, 255]
        # milansc: ignoring health for now
        #self.health = 0

        self.KM = None
        self.CB = simpleColorBalanceClass()

    def setScaleShift(self, scale, shift):
        self.scale = scale
        self.shift = shift


    def setupKM(self, numCenters, blurAlg, resize, blurKer):
        self.KM = kMeansClass(numCenters, blurAlg, resize, blurKer)


    def calculateColorBalanceThreshold(self, img, CBpercent):
        # init colorBalanceClass
        self.ThLow, self.ThHi = self.CB.thresholdAnalysis(img, CBpercent)

    def calculateTransform(self, img):
        # apply KMeans
        self.KM.applyKM(img)

        # get the indices of the matched centers
        idxBlack, idxRed, idxYellow, idxWhite = self.KM.determineColor(True, self.KM.trained_centers)

        # get centers with red
        trained_centers = np.array([self.KM.trained_centers[idxBlack], self.KM.trained_centers[idxRed],
                                    self.KM.trained_centers[idxYellow], self.KM.trained_centers[idxWhite]])

        # TODO take true centers from global variable
        true_centers = np.vstack([[70, 50, 60], [50, 70, 240], [60, 240, 230], [250, 250, 250]])

        outlierIndex, outlierCenter, averageError = self.KM.detectOutlier(trained_centers, true_centers)

        # print('average error: ' + str(averageError))

        if averageError <= 200:

            centers_name = ['black', 'red', 'yellow', 'white']
            # print('idx of detected outlier: ' + str(centers_name[outlierIndex]))

            true_centers_woOutlier = np.delete(true_centers, outlierIndex, 0)
            trained_centers_woOutlier = np.delete(trained_centers, outlierIndex, 0)


            # calculate transform with 3 centers
            T3 = calcTransform(3, trained_centers_woOutlier, true_centers_woOutlier)
            T3.calcTransform()

            self.shift = T3.shift
            self.scale = T3.scale
        else:
            rospy.loginfo('ai: average error too large. transform NOT updated.')


    def applyTransform(self, image):
        corrected_image = scaleandshift(image, self.scale, self.shift)
        corrected_image_clipped = np.clip(
            corrected_image, 0, 255).astype(np.uint8)
        return corrected_image_clipped

    def applyColorBalance(self, img, ThLow, ThHi):
        corrected_image = self.CB.applyTrafo(img, ThLow, ThHi)
        return corrected_image













