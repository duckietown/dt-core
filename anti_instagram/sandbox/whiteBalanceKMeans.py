#!/usr/bin/env python
import sys
import os#!/usr/bin/env python
import sys
import os
import numpy as np
from anti_instagram.scale_and_shift import *
from anti_instagram.kmeans_rebuild import *

def determineWhiteCluster(trained_centers):
    num_centers = len(trained_centers)
    errorWhite = np.zeros(num_centers)
    trueWhite = [255, 255, 255]
    for i in range(num_centers):
        print(trained_centers[i])
        errorWhite[i] = np.linalg.norm(trueWhite - trained_centers[i])

    whiteClusterIdx = np.argmin(errorWhite)

    return whiteClusterIdx

def estimateScale(whiteCenter):
    trueWhite = [255, 255, 255]
    scaleB = 255 / whiteCenter[0]
    scaleG = 255 / whiteCenter[1]
    scaleR = 255 / whiteCenter[2]

    return scaleB, scaleG, scaleR

def scalePicture(img, scaleB, scaleG, scaleR):
    out = scaleandshift2(img, [scaleB, scaleG, scaleR], [0, 0, 0])
    return out

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

    KM = kMeansClass(args.n_centers, args.blur, args.resize, args.blur_kernel)
    KM.applyKM(img)
    whiteIdx = determineWhiteCluster(KM.trained_centers)
    print "The determined white cluster is: " + str(KM.trained_centers[whiteIdx])
    scaleB, scaleG, scaleR = estimateScale(KM.trained_centers[whiteIdx])
    print "The scales BGR are: " + str(scaleB) + ", " + str(scaleG) + ", " + str(scaleR)
    out = scalePicture(img, scaleB, scaleG, scaleR)

    out_clipped = np.clip(out, 0, 255).astype(np.uint8)

    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', out_clipped)
    cv2.namedWindow('original', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('original', img)
    cv2.waitKey(0)

if __name__ == '__main__':
    sys.exit(main())

