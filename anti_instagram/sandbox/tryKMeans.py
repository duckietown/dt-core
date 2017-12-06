#!/usr/bin/env python
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
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


print(kmc.cluster_centers_[1]/255)
str0 = tuple([kmc.cluster_centers_[0,2],kmc.cluster_centers_[0,1],kmc.cluster_centers_[0,0]])
str1 = tuple([kmc.cluster_centers_[1,2],kmc.cluster_centers_[1,1],kmc.cluster_centers_[1,0]])
str2 = tuple([kmc.cluster_centers_[2,2],kmc.cluster_centers_[2,1],kmc.cluster_centers_[2,0]])
str3 = tuple([kmc.cluster_centers_[3,2],kmc.cluster_centers_[3,1],kmc.cluster_centers_[3,0]])
str4 = tuple([kmc.cluster_centers_[4,2],kmc.cluster_centers_[4,1],kmc.cluster_centers_[4,0]])
str5 = tuple([kmc.cluster_centers_[5,2],kmc.cluster_centers_[5,1],kmc.cluster_centers_[5,0]])
print(str1)
image0 = np.zeros((200, 200, 3), np.uint8)
image0[:] = str0
image1 = np.zeros((200, 200, 3), np.uint8)
image1[:] = str1
image2 = np.zeros((200, 200, 3), np.uint8)
image2[:] = str2
image3 = np.zeros((200, 200, 3), np.uint8)
image3[:] = str3
image4 = np.zeros((200, 200, 3), np.uint8)
image4[:] = str4
image5 = np.zeros((200, 200, 3), np.uint8)
image5[:] = str5

labelArray = kmc.labels_

num0 = np.sum(labelArray==0)
num1 = np.sum(labelArray==1)
num2 = np.sum(labelArray==2)
num3 = np.sum(labelArray==3)
num4 = np.sum(labelArray==4)
num5 = np.sum(labelArray==5)



f, axarr = plt.subplots(3, 2)
axarr[0,0].imshow(image0)
axarr[0,0].axis('off')
axarr[0,0].set_title(str(num0))

axarr[0,1].imshow(image1)
axarr[0,1].axis('off')
axarr[0,1].set_title(str(num1))

axarr[1,0].imshow(image2)
axarr[1,0].axis('off')
axarr[1,0].set_title(str(num2))

axarr[1,1].imshow(image3)
axarr[1,1].axis('off')
axarr[1,1].set_title(str(num3))

axarr[2,0].imshow(image4)
axarr[2,0].axis('off')
axarr[2,0].set_title(str(num4))

axarr[2,1].imshow(image5)
axarr[2,1].axis('off')
axarr[2,1].set_title(str(num5))


plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()

for i in range(kmc.n_clusters):
    print(np.sum(labelArray==i))

