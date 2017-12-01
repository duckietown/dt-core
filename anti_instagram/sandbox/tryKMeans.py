#!/usr/bin/env python
import sys
import cv2
import numpy as np
from sklearn.cluster import KMeans
from collections import Counter

def getimgdatapts(cv2img):
    x, y, p = cv2img.shape
    cv2_tpose = cv2img.transpose()
    cv2_arr_tpose = np.reshape(cv2_tpose,[p,x*y])
    npdata = np.transpose(cv2_arr_tpose);
    return npdata

def batchExtraction(image, batchSideLength):
    xSize, ySize, zSize = image.shape
    xSizeNew = int(xSize / batchSideLength)
    ySizeNew = int(ySize / batchSideLength)
    newImage = np.zeros((xSizeNew,ySizeNew,zSize))
    for i in range(xSizeNew):
        for j in range(ySizeNew):
            # create indices for the batches
            xlow = i*batchSideLength
            xhigh = (i+1)*batchSideLength
            ylow = j*batchSideLength
            yhigh = (j+1)*batchSideLength
            if(i == (xSizeNew-1) ):
                xhigh = xSize - 1
            if(j == (ySizeNew - 1)):
                yhigh = ySize -1
            # average the batches
            newImage[i, j, 0] = np.mean(image[xlow:xhigh, ylow:yhigh, 0])
            newImage[i, j, 1] = np.mean(image[xlow:xhigh, ylow:yhigh, 1])
            newImage[i, j, 2] = np.mean(image[xlow:xhigh, ylow:yhigh, 2])
    return newImage


input_img = cv2.imread("test_images/pic3.jpg", cv2.IMREAD_UNCHANGED)
#input_img_converted = getimgdatapts(input_img)
#print(input_img_converted.shape)

width, height, channels = input_img.shape
trial = cv2.resize(input_img, (0, 0), fx=0.1, fy=0.1)
print(trial.shape)

# blur image using gaussian:
blurG = cv2.GaussianBlur(trial, (5,5), 0)

# blur image using median:
blurM = cv2.medianBlur(trial, 5)

# plot both blurred images
blurBoth = np.concatenate((blurG, blurM), axis=1)
cv2.namedWindow('blurred Images', cv2.WINDOW_NORMAL)
cv2.imshow('blurred Images', blurBoth)
cv2.waitKey(0)
cv2.destroyAllWindows()

# apply kmeans on blurred image:

# number of centers for kmeans
n_centers = 6
kmc = KMeans(n_clusters=n_centers, init='k-means++', max_iter=20)
trial_converted = getimgdatapts(blurM)
kmc.fit(trial_converted)
trained_centers = kmc.cluster_centers_
labels = kmc.labels_

# print centers and counts
labelcount = Counter()
for i in np.arange(n_centers):
    labelcount[i] = np.sum(labels == i)
print(labelcount)
print(trained_centers)
