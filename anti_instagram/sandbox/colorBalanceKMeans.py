#!/usr/bin/env python
import sys
import os#!/usr/bin/env python
import sys
import os
import numpy as np
from anti_instagram.scale_and_shift import *
from anti_instagram.kmeans_rebuild import *

def determineErrorCluster(trained_centers, true_cluster):
    num_centers = len(trained_centers)
    errorWhite = np.zeros(num_centers)
    for i in range(num_centers):
        print(trained_centers[i])
        errorWhite[i] = np.linalg.norm(true_cluster - trained_centers[i])

    lowestErrorIdx = np.argmin(errorWhite)

    return lowestErrorIdx

def estimateScale(centers, true_cluster):
    scaleB = true_cluster[0] / centers[0]
    scaleG = true_cluster[1] / centers[1]
    scaleR = true_cluster[2] / centers[2]

    return scaleB, scaleG, scaleR

def estimateShift(centers, true_cluster):
    shiftB = true_cluster[0] - centers[0]
    shiftG = true_cluster[1] - centers[1]
    shiftR = true_cluster[2] - centers[2]

    return shiftB, shiftG, shiftR

def scalePicture(img, scaleB, scaleG, scaleR):
    out = scaleandshift2(img, [scaleB, scaleG, scaleR], [0, 0, 0])
    return out

def shiftPicture(img, shiftB, shiftG, shiftR):
    out = scaleandshift2(img, [1, 1, 1], [shiftB, shiftG, shiftR])
    return out

def blackBalance(img):
    true_cluster = [60, 60, 60]
    KM = kMeansClass(4, 'median', 0.1, 5)
    KM.applyKM(img)
    lowErrorIdx = determineErrorCluster(KM.trained_centers, true_cluster)
    shiftB, shiftG, shiftR = estimateShift(KM.trained_centers[lowErrorIdx], true_cluster)
    outShifted = shiftPicture(img, shiftB, shiftG, shiftR)
    outShifted_clipped = np.clip(outShifted, 0, 255).astype(np.uint8)
    return outShifted_clipped, shiftB, shiftG, shiftR

def main():
    # define input arguments

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

    img = cv2.imread(args.img_path, cv2.IMREAD_UNCHANGED)
    true_cluster = [60, 60, 60]
    KM = kMeansClass(args.n_centers, args.blur, args.resize, args.blur_kernel)
    KM.applyKM(img)
    lowErrorIdx = determineErrorCluster(KM.trained_centers, true_cluster)
    print "The determined white cluster is: " + str(KM.trained_centers[lowErrorIdx])
    scaleB, scaleG, scaleR = estimateScale(KM.trained_centers[lowErrorIdx], true_cluster)
    shiftB, shiftG, shiftR = estimateShift(KM.trained_centers[lowErrorIdx], true_cluster)
    print "The scales BGR are: " + str(scaleB) + ", " + str(scaleG) + ", " + str(scaleR)

    outScaled = scalePicture(img, scaleB, scaleG, scaleR)
    outShifted = shiftPicture(img, shiftB, shiftG, shiftR)

    outScaled_clipped = np.clip(outScaled, 0, 255).astype(np.uint8)
    outShifted_clipped = np.clip(outShifted, 0, 255).astype(np.uint8)

    cv2.namedWindow('scaled', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('scaled', outScaled_clipped)
    cv2.namedWindow('shifted', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('shifted', outShifted_clipped)
    cv2.namedWindow('original', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('original', img)
    cv2.waitKey(0)

if __name__ == '__main__':
    sys.exit(main())

