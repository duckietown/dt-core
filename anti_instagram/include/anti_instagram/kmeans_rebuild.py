#!/usr/bin/env python
import sys
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
from matplotlib.patches import Rectangle
from sklearn.cluster import KMeans
from collections import Counter
from anti_instagram.geom import processGeom2
import math

CENTERS_BRYW = np.array([[60, 60, 60], [60, 60, 240], [50, 240, 240], [240, 240, 240]]);
CENTERS_BYW = np.array([[60, 60, 60], [50, 240, 240], [240, 240, 240]])


class kMeansClass:
    """ This class gives the ability to use the kMeans alg. with different numbers of initial centers """
    input_image = []
    resized_image = []
    blurred_image = []
    image_array = []
    num_centers = -1
    blur_alg = []
    fac_resize = -1
    blur_kernel = -1
    trained_centers = []
    labels = []
    labelcount = Counter()
    color_array = []
    color_image_array = []

    # initialize
    def __init__(self, numCenters, blurAlg, resize, blurKer):
        # read the image
        #self.input_image = cv2.imread(inputImage, cv2.IMREAD_UNCHANGED)
	self.input_image = None
        self.num_centers = int(numCenters)
        self.blur_alg = blurAlg
        self.fac_resize = float(resize)
        self.blur_kernel = int(blurKer)
        # set up array for center colors
        self.color_image_array = np.zeros((self.num_centers, 200, 200, 3), np.uint8)

    # re-shape input image for kMeans
    def _getimgdatapts(self, cv2img, fancyGeom=False):
        x, y, p = cv2img.shape
	if not fancyGeom:
            img_geom = cv2img[int(x * 0.3):(x - 1), :, :]
	    x_new, y_new, p = img_geom.shape
	    cv2_tpose = img_geom.transpose()
	    cv2_arr_tpose = np.reshape(cv2_tpose, [p, x_new * y_new])
	else:
	    mask = processGeom2(cv2img)
	    img_geom = np.expand_dims(mask,axis=-1)*cv2img
	    mask = mask.transpose()
	    inds = np.array(np.nonzero(mask))
	    cv2_tpose = np.transpose(img_geom)
	    cv2_arr_tpose = cv2_tpose[:,inds[0,:],inds[1,:]]
        npdata = np.transpose(cv2_arr_tpose)
        return npdata

    def _blurImg(self):
        # blur image using median:
        if self.blur_alg == 'median':
            self.blurred_image = cv2.medianBlur(self.resized_image, self.blur_kernel)

        # blur image using gaussian:
        elif self.blur_alg == 'gaussian':
            self.blurred_image = cv2.GaussianBlur(self.resized_image, (self.blur_kernel, self.blur_kernel), 0)
	
	else:
	    self.blurred_image = self.resized_image

    # apply kMeans alg
    def applyKM(self, img, fancyGeom=False):
	self.input_image = img
        # resize image
        self.resized_image = cv2.resize(self.input_image, (0, 0), fx=self.fac_resize, fy=self.fac_resize)

        # blur image
        self._blurImg()

        # prepare KMeans
        kmc = KMeans(n_clusters=self.num_centers, init='k-means++', max_iter=20)

        # prepare data points
        self.image_array = self._getimgdatapts(self.blurred_image, fancyGeom=fancyGeom)

        # run KMeans
        kmc.fit(self.image_array)

        # get centers, labels and labelcount from KMeans
        self.trained_centers = kmc.cluster_centers_
        self.labels = kmc.labels_
        for i in np.arange(self.num_centers):
            self.labelcount[i] = np.sum(self.labels == i)

    def determineColor(self, withRed, trained_centers):

        # define the true centers. This color is preset. The color transformation
        # tries to transform a picture such that the black areas will become true black.
        # The same applies for yellow, white and (if valid) red.
        trueBlack = [60, 60, 60]
        trueYellow = [50, 240, 240]
        trueWhite = [240, 240, 240]
        if (withRed):
            trueRed = [60, 60, 240]

        # initialize arrays which save the errors to each true center
        # later the minimal error cluster center will be defined as this color
        errorBlack = np.zeros(self.num_centers)
        errorYellow = np.zeros(self.num_centers)
        errorWhite = np.zeros(self.num_centers)
        if (withRed):
            errorRed = np.zeros(self.num_centers)

        # determine the error for each trained cluster center to all true centers
        for i in range(self.num_centers):
            errorBlack[i] = np.linalg.norm(trueBlack - trained_centers[i])
            errorYellow[i] = np.linalg.norm(trueYellow - trained_centers[i])
            errorWhite[i] = np.linalg.norm(trueWhite - trained_centers[i])
            if (withRed):
                errorRed[i] = np.linalg.norm(trueRed - trained_centers[i])


        nTrueCenters = 3

        # sort the error arrays and save the corresponding index of the original array
        # in the following array. This allows us to determine the index of the cluster.
        errorBlackSortedIdx = np.argsort(errorBlack)
        errorYellowSortedIdx = np.argsort(errorYellow)
        errorWhiteSortedIdx = np.argsort(errorWhite)
	errorSorted = np.vstack([errorBlack, errorWhite, errorYellow])
        #print(errorSorted)
	if (withRed):
            errorRedSortedIdx = np.argsort(errorRed)
	    errorSorted = np.vstack((errorSorted,errorRed))
        if (withRed):
            nTrueCenters = 4
        ListOfIndices = []

        # boolean variables to determine whether the minimal error index has been found
        blackIdxFound = False
        whiteIdxFound = False
        yellowIdxFound = False
        if (withRed):
            redIdxFound = False
        centersFound = False
        index = 0

	w,h = errorSorted.shape
	errorList = np.reshape(errorSorted,(w*h))
        # find for every true center the corresponding trained center.
	#this code considers the global minimum for assigning clusters,
	#instead of assigning first black, then white, yellow and red
        while (not centersFound):
	    ind = np.argmin(errorList)
	    xi,yi = ind//h, ind%h
	    if xi==0 and not blackIdxFound:
		ListOfIndices.append(yi)
		blackIdxFound = True
		idxBlack = yi
	    if xi==1 and not whiteIdxFound:
	        ListOfIndices.append(yi)
		whiteIdxFound = True
		idxWhite = yi
	    if xi==2 and not yellowIdxFound:
	        ListOfIndices.append(yi)
		yellowIdxFound = True
		idxYellow = yi
	    if (withRed):
		if xi==3 and not redIdxFound:
		    ListOfIndices.append(yi)
		    redIdxFound = True
		    idxRed = yi
		centersFound = blackIdxFound and whiteIdxFound and yellowIdxFound and redIdxFound
	    else:
		centersFound = blackIdxFound and whiteIdxFound and yellowIdxFound
	    errorSorted[xi,:] = np.max(errorSorted)
	    errorSorted[:,yi] = np.max(errorSorted)
	    errorList = np.reshape(errorSorted,(w*h))
            #if errorBlackSortedIdx[index] not in ListOfIndices and not blackIdxFound:
            #    ListOfIndices.append(errorBlackSortedIdx[index])
            #    blackIdxFound = True
            #    idxBlack = errorBlackSortedIdx[index]
            #if errorWhiteSortedIdx[index] not in ListOfIndices and not whiteIdxFound:
            #    ListOfIndices.append(errorWhiteSortedIdx[index])
            #    whiteIdxFound = True
            #    idxWhite = errorWhiteSortedIdx[index]
            #if errorYellowSortedIdx[index] not in ListOfIndices and not yellowIdxFound:
            #    ListOfIndices.append(errorYellowSortedIdx[index])
            #    yellowIdxFound = True
            #    idxYellow = errorYellowSortedIdx[index]
            #if withRed:
            #    if errorRedSortedIdx[index] not in ListOfIndices and not redIdxFound:
            #        ListOfIndices.append(errorRedSortedIdx[index])
            #        redIdxFound = True
            #        idxRed = errorRedSortedIdx[index]
            #    centersFound = blackIdxFound and whiteIdxFound and yellowIdxFound and redIdxFound

            #else:
            #    centersFound = blackIdxFound and whiteIdxFound and yellowIdxFound
            #index = index + 1

        # return the minimal error indices for the trained centers.
        if (withRed):
            return idxBlack, idxRed, idxYellow, idxWhite,
        else:
            return idxBlack, idxYellow, idxWhite
