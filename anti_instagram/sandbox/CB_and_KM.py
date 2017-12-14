#!/usr/bin/env python
from wb2trafo import *
from anti_instagram.AntiInstagram_rebuild import *
from simpleColorBalance import *
import cv2
import argparse
import os
import sys
import numpy as np


"""
This script tries to find an approximation for the trafo of the autoColorBalance.
It then applies the kmeans alg on the output of the autoCB, and finds the superposition
of these two trafos.
"""

# if False --> use Shift
SCALE = False

def main():
    # define and parse command line arguments
    parser = argparse.ArgumentParser(
        description='lorem Ipsum.')
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
    parser.add_argument('--percentage', default='1',
                        help='percentage for WB. DEFAULT = 1')
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
    if not (args.blur == "median" or args.blur == "gaussian" or args.blur == "none"):
        print('blur alg must be none, median or gaussian')
        sys.exit(2)

    # check kernel size
    if (int(args.blur_kernel) % 2 == 0):
        print('kernel size must be odd')
        sys.exit(2)

    # read image
    image_cv = cv2.imread(args.img_path, cv2.IMREAD_UNCHANGED)

    # blur if desired
    image_blurred = blurImg(image_cv, args.blur, int(args.blur_kernel))

    # apply simple color balance
    image_cb = simplest_cb(image_blurred, int(args.percentage))

    # get average colors
    avg_input = averageColor(image_cv)
    print('average color of input image:\n' + str(avg_input))
    avg_cb = averageColor(image_cb)
    print('average color of output image:\n' + str(avg_cb))

    # get shift
    shift = getShift(avg_input, avg_cb)
    print('shift:\n' + str(shift))

    # get scale
    scale = getScale(avg_input, avg_cb)
    print('scale:\n' + str(scale))

    # create instance of anti instagram
    ai = AntiInstagram()
    ai.setupKM(args.n_centers, args.blur, args.resize, args.blur_kernel)

    # apply kmeans on color balanced image
    ai.calculateTransform(image_cb)

    km_shift = ai.shift
    km_scale = ai.scale

    if SCALE:
        ai.setScaleShift(km_scale, [0, 0, 0])
        image_corr1 = ai.applyTransform(image_cv)
        ai.setScaleShift(scale, [0, 0, 0])
        image_corr2 = ai.applyTransform(image_corr1)

    else:
        ai.setScaleShift([1, 1, 1], km_shift)
        image_corr1 = ai.applyTransform(image_cv)
        ai.setScaleShift([1, 1, 1], shift)
        image_corr2 = ai.applyTransform(image_corr1)







    compare = np.concatenate((image_cv, image_cb), axis=1)

    compare2 = np.concatenate((compare, image_corr2), axis=1)

    cv2.namedWindow('input vs ColorBalance vs combined', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('input vs ColorBalance vs combined', compare2)


    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())