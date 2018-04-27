#!/usr/bin/env python
from simpleColorBalance import *
import cv2
import argparse
import os
import sys
import numpy as np
from anti_instagram.AntiInstagram_rebuild import *



def blurImg(img, method, kernel):
    # blur image using median:
    if method == 'median':
        blurred_image = cv2.medianBlur(img, kernel)

    # blur image using gaussian:
    elif method == 'gaussian':
        blurred_image = cv2.GaussianBlur(img, (kernel, kernel), 0)

    # no blur
    elif method == 'none':
        blurred_image = img.copy()

    return blurred_image


def averageColor(img):
    avg_color_per_row = np.average(img, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)

    return avg_color

def getShift(input_avg, output_avg):
    shift = [0, 0, 0]
    for i in range(3):
        shift[i] = output_avg[i] - input_avg[i]
    return shift

def getScale(input_avg, output_avg):
    scale = [0, 0, 0]
    for i in range(3):
        scale[i] = output_avg[i] / input_avg[i]
    return scale


def main():
    # define and parse command line arguments
    parser = argparse.ArgumentParser(
        description='lorem Ipsum.')
    parser.add_argument('img_path', help='path to the image')
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



    image_cb = simplest_cb(image_blurred, int(args.percentage))





    avg_input = averageColor(image_cv)
    print('average color of input image:\n' + str(avg_input))

    avg_cb = averageColor(image_cb)
    print('average color of output image:\n' + str(avg_cb))

    shift = getShift(avg_input, avg_cb)
    print('shift:\n' + str(shift))

    scale = getScale(avg_input, avg_cb)
    print('scale:\n' + str(scale))




    ai = AntiInstagram()

    ai.setScaleShift(scale, [0, 0, 0])
    scaled_image = ai.applyTransform(image_cv)

    ai.setScaleShift([1, 1, 1], shift)
    shifted_image = ai.applyTransform(image_cv)




    compare = np.concatenate((image_cv, image_cb), axis=1)

    compare2 = np.concatenate((scaled_image, shifted_image), axis=1)

    cv2.namedWindow('input vs ColorBalance', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('input vs ColorBalance', compare)

    cv2.namedWindow('scaled vs shifted', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('scaled vs shifted', compare2)

    cv2.waitKey(0)
    cv2.destroyAllWindows()





if __name__ == '__main__':
    sys.exit(main())