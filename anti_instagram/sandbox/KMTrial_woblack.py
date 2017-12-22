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
from anti_instagram.calcLstsqTransform import *
from anti_instagram.AntiInstagram import *
from anti_instagram.scale_and_shift import *
# from .scale_and_shift import scaleandshift
# from .scale_and_shift import scaleandshift2
from simpleColorBalance import *
from colorBalanceKMeans import *

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
        self.shiftB = None
        self.shiftG = None
        self.shiftR = None
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
        img_geom = cv2img[int(x*0):(x-1), :, :]
        x_new, y_new, p = img_geom.shape
        cv2_tpose = img_geom.transpose()
        cv2_arr_tpose = np.reshape(cv2_tpose, [p, x_new * y_new])
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
            color_i = tuple(
                [self.trained_centers[center, 2], self.trained_centers[center, 1], self.trained_centers[center, 0]])
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
                    axarr[row, 0].imshow(self.color_image_array[2 * row])
                    axarr[row, 0].axis('off')
                    axarr[row, 0].set_title(str(self.labelcount[2 * row]))

                    axarr[row, 1].imshow(self.color_image_array[2 * row + 1])
                    axarr[row, 1].axis('off')
                    axarr[row, 1].set_title(str(self.labelcount[2 * row + 1]))
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

        # blur image
        self._blurImg()
        print('blurred image!')
        self.blurred_image, self.shiftB, self.shiftG, self.shiftR = blackBalance(self.blurred_image)
        # prepare KMeans
        kmc = KMeans(n_clusters=self.num_centers, init='k-means++', max_iter=20)


        # try out color balance first
        # self.blurred_image = simplest_cb(self.blurred_image, 1) # percentages around 1% are normal
        cv2.namedWindow('blurred', flags=cv2.WINDOW_NORMAL)
        cv2.imshow('blurred', self.blurred_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # prepare data points
        self.image_array = self._getimgdatapts(self.blurred_image)

        # debug
        print(self.image_array.shape)
        # run KMeans
        kmc.fit(self.image_array)

        # get centers, labels and labelcount from KMeans
        self.trained_centers = kmc.cluster_centers_
        self.labels = kmc.labels_
        for i in np.arange(self.num_centers):
            self.labelcount[i] = np.sum(self.labels == i)

        # plot colors
        self._plotColors()

    def determineColor(self, withRed, trained_centers):

        # define the true centers. This color is preset. The color transformation
        # tries to transform a picture such that the black areas will become true black.
        # The same applies for yellow, white and (if valid) red.
        if (withRed):
            trueRed = [60, 60, 240]
        trueYellow = [50, 240, 240]
        trueWhite = [240, 240, 240]

        errorYellow = np.zeros(self.num_centers)
        errorWhite = np.zeros(self.num_centers)

        if (withRed):
            errorRed = np.zeros(self.num_centers)

        for i in range(self.num_centers):
            print(trained_centers[i])
            errorYellow[i] = np.linalg.norm(trueYellow - trained_centers[i])
            errorWhite[i] = np.linalg.norm(trueWhite - trained_centers[i])
            if (withRed):
                errorRed[i] = np.linalg.norm(trueRed - trained_centers[i])

        print "yellow error:" + str(errorYellow)
        print "white error:" + str(errorWhite)
        print "red error:" + str(errorRed)
        nTrueCenters = 3
        errorYellowSortedIdx = np.argsort(errorYellow)
        errorWhiteSortedIdx = np.argsort(errorWhite)
        if (withRed):
            errorRedSortedIdx = np.argsort(errorRed)
        if (withRed):
            nTrueCenters = 4
        ListOfIndices = []

        whiteIdxFound = False
        yellowIdxFound = False
        if (withRed):
            redIdxFound = False
        centersFound = False
        index = 0

        print "errorYellowSortedIdx: " + str(errorYellowSortedIdx)
        print "errorWhiteSortedIdx: " + str(errorWhiteSortedIdx)
        print "errorRedSortedIdx: " + str(errorRedSortedIdx)
        while (not centersFound):

            if errorWhiteSortedIdx[index] not in ListOfIndices and not whiteIdxFound:
                ListOfIndices.append(errorWhiteSortedIdx[index])
                print str(index) + " in white " + str(ListOfIndices)
                whiteIdxFound = True
                idxWhite = errorWhiteSortedIdx[index]
            if errorYellowSortedIdx[index] not in ListOfIndices and not yellowIdxFound:
                ListOfIndices.append(errorYellowSortedIdx[index])
                print str(index) + " in yellow " + str(ListOfIndices)
                yellowIdxFound = True
                idxYellow = errorYellowSortedIdx[index]
            if withRed:
                if errorRedSortedIdx[index] not in ListOfIndices and not redIdxFound:
                    ListOfIndices.append(errorRedSortedIdx[index])
                    redIdxFound = True
                    print str(index) + "in red" + str(ListOfIndices)
                    idxRed = errorRedSortedIdx[index]
                print "True?: " + str(redIdxFound) + str(yellowIdxFound) + str(whiteIdxFound)
                centersFound = whiteIdxFound and yellowIdxFound and redIdxFound
                print "centersFound: " + str(centersFound)

            else:
                centersFound = whiteIdxFound and yellowIdxFound
            index = index + 1
            print "End of while loop. Index: " + str(index)

        print idxRed, idxWhite, idxYellow
        if (withRed):
            return idxRed, idxYellow, idxWhite
        else:
            return idxYellow, idxWhite


    def plotDeterminedCenters(self, centerYellow, centerWhite, centerRed):

        tupleWhite = tuple([centerWhite[2], centerWhite[1], centerWhite[0]])
        tupleYellow = tuple([centerYellow[2], centerYellow[1], centerYellow[0]])
        tupleRed = tuple([centerRed[2], centerRed[1], centerRed[0]])

        imageWhite = np.zeros((200, 200, 3), np.uint8)
        imageWhite[:] = tupleWhite
        imageYellow = np.zeros((200, 200, 3), np.uint8)
        imageYellow[:] = tupleYellow
        imageRed = np.zeros((200, 200, 3), np.uint8)
        imageRed[:] = tupleRed

        f, axarr = plt.subplots(2, 2)

        axarr[0, 1].imshow(imageWhite)
        axarr[0, 1].axis('off')
        axarr[0, 1].set_title("White")

        axarr[1, 0].imshow(imageYellow)
        axarr[1, 0].axis('off')
        axarr[1, 0].set_title("Yellow")

        axarr[1, 1].imshow(imageRed)
        axarr[1, 1].axis('off')
        axarr[1, 1].set_title("Red")

        plt.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()

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
    print "kernel: " + str(args.blur_kernel)
    if (int(args.blur_kernel) % 2 == 0):
        print('kernel size must be odd')
        sys.exit(2)

    # create instance of kMeans
    print("all arguments have been read.")

    KM = kMeanClass(args.img_path, args.n_centers, args.blur, args.resize, args.blur_kernel)
    cv2.namedWindow('input', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('input', KM.input_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    KM.applyKM()
    idxRed, idxYellow, idxWhite  = KM.determineColor(True, KM.trained_centers)
    KM.plotDeterminedCenters(KM.trained_centers[idxYellow],
                             KM.trained_centers[idxWhite], KM.trained_centers[idxRed])

    trained_centers = np.array([KM.trained_centers[idxRed], KM.trained_centers[idxYellow], KM.trained_centers[idxWhite]])
    trained_centers_woRed = np.array([KM.trained_centers[idxYellow], KM.trained_centers[idxWhite]])

    print(trained_centers)

    print("transform instance will be created!")
    T = calcTransform(3, trained_centers, np.array([[60, 60, 240], [50, 240, 240], [240, 240, 240]]))
    T.calcTransform()

    corr_img1 = scaleandshift2(KM.input_image, [1, 1, 1], [KM.shiftB, KM.shiftG, KM.shiftR])
    corrected_img = scaleandshift2(corr_img1, T.scale, T.shift)
    corrected_image_cv2 = np.clip(corrected_img, 0, 255).astype(np.uint8)

    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', corrected_image_cv2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())

