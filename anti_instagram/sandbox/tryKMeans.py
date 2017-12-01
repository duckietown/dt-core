#!/usr/bin/env python
import sys
import cv2
import numpy as np
from sklearn.cluster import KMeans

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


input_img = cv2.imread("test_images/pic1.jpg", cv2.IMREAD_UNCHANGED)
input_img_converted = getimgdatapts(input_img)
print(input_img_converted.shape)
trial = batchExtraction(input_img, 9)
print(trial.shape)
kmc = KMeans(n_clusters=10, init='k-means++', max_iter=20)
cv2.imshow('image', trial)
cv2.waitKey(0)
cv2.destroyAllWindows()
#kmc.fit(input_img_converted)
#print(kmc.cluster_centers_)
