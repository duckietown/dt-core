#!/usr/bin/env python
# from anti_instagram.simpleColorBalanceClass import *
from anti_instagram.AntiInstagram_rebuild import *
import cv2
import sys
import os
import argparse
import time

def main():
    # define and parse command line arguments

    parser = argparse.ArgumentParser(
        description='Perform kMeans with n initial centers.')
    parser.add_argument('img_path', help='path to the image')
    parser.add_argument('n_centers', help='numbers of initial centers')
    parser.add_argument('percentage', help='percent of cut-off')
    parser.add_argument('--resize', default='1',
                        help='factor of downsampling the input image. DEFAULT = 1')
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

    # check if other file exists
    #if not os.path.isfile(args.img_path_2):
    #    print('file not found')
    #    sys.exit(2)

    # check if dir exists, create if not
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)




    # read image
    image_cv_1 = cv2.imread(args.img_path, cv2.IMREAD_UNCHANGED)

    # read other image
    #image_cv_2 = cv2.imread(args.img_path_2, cv2.IMREAD_UNCHANGED)

    ai = AntiInstagram()
    ai.setupKM(int(args.n_centers), args.blur, float(args.resize), int(args.blur_kernel))
    start = time.time()
    ai.calculateColorBalanceThreshold(image_cv_1, int(args.percentage))
    time1 = time.time()
    colorBalanced_image = ai.applyColorBalance(img=image_cv_1, ThLow=ai.ThLow, ThHi=ai.ThHi)
    time2 = time.time()
    ai.calculateTransform(colorBalanced_image)
    time3 = time.time()
    corrected_image = ai.applyTransform(colorBalanced_image)
    time4 = time.time()

    print('color balance analysis took: ' + str(time1-start))
    print('apply color balance took: ' + str(time2-time1))
    print('color trafo analysis took: ' + str(time3-time2))
    print('apply color trafo took: ' + str(time4-time3))


    compare1 = np.concatenate((image_cv_1, colorBalanced_image), axis=1)
    compare2 = np.concatenate((compare1, corrected_image), axis=1)
    cv2.namedWindow('corrected', flags=cv2.WINDOW_NORMAL)
    cv2.imshow('corrected', compare2)

    cv2.waitKey(0)
    cv2.destroyAllWindows()






if __name__ == '__main__':
    sys.exit(main())