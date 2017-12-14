#!/usr/bin/env python
from anti_instagram.kmeans_rebuild import *
from anti_instagram.calcLstsqTransform import *
from .scale_and_shift import scaleandshift


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
        self.scale = [1.0, 1.0, 1.0]
        self.shift = [0.0, 0.0, 0.0]
        # milansc: ignoring health for now
        #self.health = 0

        self.KM = None

    def setScaleShift(self, scale, shift):
        self.scale = scale
        self.shift = shift


    def setupKM(self, numCenters, blurAlg, resize, blurKer):
        self.KM = kMeansClass(numCenters, blurAlg, resize, blurKer)


    def calculateTransform(self, img):
        # apply KMeans
        self.KM.applyKM(img)

        # get the indices of the matched centers
        idxBlack, idxRed, idxYellow, idxWhite = self.KM.determineColor(True, self.KM.trained_centers)

        # get centers with red
        trained_centers = np.array([self.KM.trained_centers[idxBlack], self.KM.trained_centers[idxRed],
                                    self.KM.trained_centers[idxYellow], self.KM.trained_centers[idxWhite]])

        # get centers w/o red
        trained_centers_woRed = np.array([self.KM.trained_centers[idxBlack], self.KM.trained_centers[idxYellow],
                                          self.KM.trained_centers[idxWhite]])

        # calculate transform with 4 centers
        T4 = calcTransform(4, trained_centers)
        T4.calcTransform()

        # calculate transform with 3 centers
        T3 = calcTransform(3, trained_centers_woRed)
        T3.calcTransform()

        # compare residuals
        # in practice, this is NOT a fair way to compare the residuals, 4 will almost always win out,
        # causing a serious red shift in any image that has only 3 colors
        if T4.returnResidualNorm() >= T3.returnResidualNorm():
            self.shift = T4.shift
            self.scale = T4.scale
        else:
            self.shift = T3.shift
            self.scale = T3.scale

        self.shift = T3.shift
        self.scale = T3.scale


    def applyTransform(self, image):
        corrected_image = scaleandshift(image, self.scale, self.shift)
        corrected_image_clipped = np.clip(
            corrected_image, 0, 255).astype(np.uint8)
        return corrected_image_clipped