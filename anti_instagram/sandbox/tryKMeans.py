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
import math


class kMeanClass:
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
    def __init__(self, inputImage, numCenters, blurAlg, resize, blurKer):
        # read the image
        self.input_image = cv2.imread(inputImage, cv2.IMREAD_UNCHANGED)
        self.num_centers = int(numCenters)
        self.blur_alg = blurAlg
        self.fac_resize = float(resize)
        self.blur_kernel = int(blurKer)
        # set up array for center colors
        self.color_image_array = np.zeros((self.num_centers, 200, 200, 3), np.uint8)
        print('created instance of kMeans with arguments:')
        print('     input image = ' + str(inputImage))
        print('     number of centers = ' + str(self.num_centers))
        print('     blur algorithm = ' + str(self.blur_alg))
        print('     resize factor = ' + str(self.fac_resize))
        print('     blurring kernel size = ' + str(self.blur_kernel))

    # re-shape input image for kMeans
    def _getimgdatapts(self, cv2img):
        x, y, p = cv2img.shape
        cv2_tpose = cv2img.transpose()
        cv2_arr_tpose = np.reshape(cv2_tpose, [p, x * y])
        npdata = np.transpose(cv2_arr_tpose)
        return npdata

    def _blurImg(self):
        # blur image using median:
        if self.blur_alg == 'median':
            self.blurred_image = cv2.medianBlur(self.resized_image, self.blur_kernel)

        # blur image using gaussian:
        elif self.blur_alg == 'gaussian':
            self.blurred_image = cv2.GaussianBlur(self.resized_image, (self.blur_kernel, self.blur_kernel), 0)

    def _plotColors(self):
        # loop over all centers
        for center in np.arange(self.num_centers):
            # get color
            color_i = tuple([self.trained_centers[center,2],self.trained_centers[center,1],self.trained_centers[center,0]])
            self.color_array.append(color_i)

            self.color_image_array[center, :] = color_i

        plotRows = int(math.ceil(self.num_centers / 2.0))
        f, axarr = plt.subplots(plotRows, 2)
        for row in range(plotRows):
            if self.num_centers % 2 == 0:
                axarr[row, 0].imshow(self.color_image_array[2 * row])
                axarr[row, 0].axis('off')
                axarr[row, 0].set_title(str(self.labelcount[2 * row]))

                axarr[row, 1].imshow(self.color_image_array[2 * row + 1])
                axarr[row, 1].axis('off')
                axarr[row, 1].set_title(str(self.labelcount[2 * row + 1]))
            else:
                if row != plotRows - 1:
                    axarr[row, 0].imshow(self.color_image_array[2*row])
                    axarr[row, 0].axis('off')
                    axarr[row, 0].set_title(str(self.labelcount[2*row]))

                    axarr[row, 1].imshow(self.color_image_array[2*row + 1])
                    axarr[row, 1].axis('off')
                    axarr[row, 1].set_title(str(self.labelcount[2*row + 1]))
                else:
                    axarr[row, 0].imshow(self.color_image_array[2 * row])
                    axarr[row, 0].axis('off')
                    axarr[row, 0].set_title(str(self.labelcount[2 * row]))

                    axarr[row, 1].axis('off')
        print(self.color_array)
        plt.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()




    # apply kMeans alg
    def applyKM(self):
        # resize image
        self.resized_image = cv2.resize(self.input_image, (0, 0), fx=self.fac_resize, fy=self.fac_resize)
        print('resized image!')
        cv2.imshow('img', self.resized_image)
        cv2.waitKey(0)

        # blur image
        self._blurImg()
        print('blurred image!')

        # prepare KMeans
        kmc = KMeans(n_clusters=self.num_centers, init='k-means++', max_iter=20)

        # prepare data points
        self.image_array = self._getimgdatapts(self.blurred_image)

        # run KMeans
        kmc.fit(self.image_array)

        # get centers, labels and labelcount from KMeans
        self.trained_centers = kmc.cluster_centers_
        self.labels = kmc.labels_
        for i in np.arange(self.num_centers):
            self.labelcount[i] = np.sum(self.labels == i)

        # plot colors
        self._plotColors()



def main():
    # define and parse command line arguments
    parser = argparse.ArgumentParser(
        description='Perform kMeans with n initial centers.')
    parser.add_argument('img_path', help='path to the image')
    parser.add_argument('n_centers', help='numbers of initial centers')
    parser.add_argument('--resize', default='0.1',
                        help='factor of downsampling the input image. DEFAULT = 0.1')
    parser.add_argument('--blur', default='median',
                        help="blur algorithm. 'median' or 'gaussian. DEFAULT = median")
    parser.add_argument('--blur_kernel', default='5',
                        help='size of kernel for blurring. DEFAULT = 5')
    parser.add_argument('--output_dir', default='./output_images',
                        help='directory for the output images. DEFAULT = ./output_images')
    args = parser.parse_args()

    # check if file exists
    if not os.path.isfile(args.img_path):
        print('file not found')
        sys.exit(2)

    # check if dir exists, create if not
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # check resize factor
    if (args.resize < 1) or (args.resize <= 0):
        print('resize factor between 0 and 1')
        sys.exit(2)

    # check blur alg
    if not (args.blur == "median" or args.blur == "gaussian"):
        print('blur alg must be median or gaussian')
        sys.exit(2)

    # check kernel size
    print(args.blur_kernel)
    if (int(args.blur_kernel) % 2 == 0):
        print('kernel size must be odd')
        sys.exit(2)

    # create instance of kMeans
    KM = kMeanClass(args.img_path, args.n_centers, args.blur, args.resize, args.blur_kernel)
    KM.applyKM()






if __name__=='__main__':
    sys.exit(main())






"""
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
"""
