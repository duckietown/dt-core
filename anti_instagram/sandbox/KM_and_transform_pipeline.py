#!/usr/bin/env python
import numpy as np

#from anti_instagram.AntiInstagram import *
from anti_instagram.scale_and_shift import *
from anti_instagram.calcLstsqTransform import *
from anti_instagram.kmeans_rebuild import *
import sys
import os
import matplotlib.pyplot as plt
import argparse

"""
This function does three things:

1)  It calls the Kmeans alg. with the desired number of initial centers. Other parameters can be tweaked, see -h option.

2)  The function passes the centers trained by the KMeans alg. to the calcTransform function. It then calculates
    a transform once based on three centers and once based on four centers (without and with the color red)

3)  It takes the transform with the better result (most frequent three centers) and applies the transform on the
    input image.
"""


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
    parser.add_argument('--fancyGeom', default=False, action='store_true',
			help='use the contour detector to find regions of interest')
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
    KM = kMeansClass(args.img_path, args.n_centers, args.blur, args.resize, args.blur_kernel, args.fancyGeom)

    # apply KMeans
    KM.applyKM()

    # get the indices of the matched centers
    idxBlack, idxRed, idxYellow, idxWhite  = KM.determineColor(True, KM.trained_centers)

    # get centers with red
    trained_centers = np.array([KM.trained_centers[idxBlack], KM.trained_centers[idxRed],
                                KM.trained_centers[idxYellow], KM.trained_centers[idxWhite]])

    # get centers w/o red
    trained_centers_woRed = np.array([KM.trained_centers[idxBlack], KM.trained_centers[idxYellow],
                                KM.trained_centers[idxWhite]])

    print(trained_centers)

    # calculate transform with 4 centers
    T4 = calcTransform(4, trained_centers)
    T4.calcTransform()

    # calculate transform with 3 centers
    T3 = calcTransform(3, trained_centers_woRed)
    T3.calcTransform()

    # compare residuals
    # TODO verify if we can compare the residuals like this
    if T4.returnResidualNorm() >= T3.returnResidualNorm():
        shift = T4.shift
        scale = T4.scale
    else:
        shift = T3.shift
        scale = T3.scale

    # apply transform
    corrected_img = scaleandshift2(KM.input_image, scale, shift)
    corrected_image_cv2 = np.clip(
        corrected_img, 0, 255).astype(np.uint8)

    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', corrected_image_cv2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())

