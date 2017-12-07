#!/usr/bin/env python
import numpy as np

from anti_instagram.AntiInstagram import *
from anti_instagram.scale_and_shift import *
from anti_instagram.calcLstsqTransform import *
from anti_instagram.kmeans_rebuild import *
import sys
import os
import matplotlib.pyplot as plt
import argparse



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
    KM = kMeansClass(args.img_path, args.n_centers, args.blur, args.resize, args.blur_kernel)
    KM.applyKM()
    idxBlack, idxRed, idxYellow, idxWhite  = KM.determineColor(True, KM.trained_centers)
    #KM.plotDeterminedCenters(KM.trained_centers[idxBlack], KM.trained_centers[idxYellow],
     #                        KM.trained_centers[idxWhite], KM.trained_centers[idxRed])

    trained_centers = np.array([KM.trained_centers[idxBlack], KM.trained_centers[idxRed], KM.trained_centers[idxYellow], KM.trained_centers[idxWhite]])
    trained_centers_woRed = np.array([KM.trained_centers[idxBlack], KM.trained_centers[idxYellow],
                                KM.trained_centers[idxWhite]])

    print(trained_centers)
    T4 = calcTransform(4, trained_centers)
    T4.calcTransform()
    T3 = calcTransform(3, trained_centers_woRed)
    T3.calcTransform()

    corrected_img_4c = scaleandshift2(KM.input_image, T4.scale, T4.shift)
    corrected_image_4c_cv2 = np.clip(
        corrected_img_4c, 0, 255).astype(np.uint8)

    corrected_img_3c = scaleandshift2(KM.input_image, T3.scale, T3.shift)
    corrected_image_3c_cv2 = np.clip(
        corrected_img_3c, 0, 255).astype(np.uint8)

    combinedImg = np.concatenate((corrected_image_4c_cv2, corrected_image_3c_cv2), axis=1)

    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', combinedImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())

