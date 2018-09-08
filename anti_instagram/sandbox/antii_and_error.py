#!/usr/bin/env python
import datetime
import os
import sys

import cv2
import numpy as np
from anti_instagram.AntiInstagram import *

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from geom import identifyLaneSurface
from error_estimation import estimateError

def main():
    # check number of arguments
    if len(sys.argv) != 3:
        print('This program expects exactly two arguments: an image filename and an output directory.')
        sys.exit()

    # store inputs
    file = sys.argv[1]
    outdir = sys.argv[2]

    # check if file exists
    if not os.path.isfile(file):
        print('file not found')
        sys.exit(2)

    # check if dir exists, create if not
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    # read the image
    input_img = cv2.imread(file, cv2.IMREAD_UNCHANGED)

    # lets do the masking!
    surf = identifyLaneSurface(input_img, use_hsv=False, grad_thresh=20)
    input_img = np.expand_dims(surf, -1) * input_img
    print('masked image!')
    cv2.imshow('mask', input_img)
    cv2.waitKey(0)

    # create instance of AntiInstagram
    ai = AntiInstagram()

    ai.calculateTransform(input_img)
    print('Transform Calculation completed!')

    corrected_img = ai.applyTransform(input_img)
    print('Transform applied!')
    cv2.imshow('mask', corrected_img)
    cv2.waitKey(0)


    #  polygons for pic1_smaller.jpg and pic2_smaller.jpg (hardcoded for now)
    polygon_black = [(280, 570), (220, 760), (870, 750), (810, 580)]
    polygon_white = [(781, 431), (975, 660), (1040, 633), (827, 418)]
    polygon_yellow = [(131, 523), (67, 597), (99, 609), (161, 530)]
    polygon_red = [(432, 282), (418, 337), (577, 338), (568, 283)]
    """
    #  polygons for pic3_smaller.jpg and pic2_smaller.jpg (hardcoded for now)
    polygon_black = [(280, 570), (220, 760), (870, 750), (810, 580)]
    polygon_white = [(900, 520), (1000, 640), (1060, 620), (970, 515)]
    polygon_yellow = [(234, 430), (190, 485), (230, 490), (270, 430)]
    polygon_red = [(285, 435), (250, 490), (830, 480), (800, 437)]


    #  polygons for pic4_smaller.jpg (hardcoded for now)
    polygon_black = [(316, 414), (215, 623), (783, 605), (673, 422)]
    polygon_white = [(710, 388), (947, 656), (1018, 620), (788, 400)]
    polygon_yellow = [(148, 474), (94, 537), (133, 542), (184, 475)]
    polygon_red = [(285, 435), (250, 490), (830, 480), (800, 437)]


    #  polygons for pic5_smaller.jpg (hardcoded for now)
    polygon_black = [(354, 418), (291, 612), (804, 590), (677, 396)]
    polygon_white = [(783, 424), (949, 602), (1002, 564), (840, 420)]
    polygon_yellow = [(344, 307), (331, 319), (354, 319), (366, 306)]
    polygon_red = [(135, 325), (119, 332), (325, 316), (332, 309)]
    """

    # create dictionary containing colors
    polygons = {'black': polygon_black, 'white': polygon_white, 'yellow': polygon_yellow, 'red': polygon_red}

    # initialize estimateError class
    E = estimateError(corrected_img)
    # set polygons
    E.createPolygon(polygons)
    # estimate error
    E.GetErrorEstimation()

    # write the corrected image
    date = datetime.datetime.now().strftime("%H-%M-%S")
    path = outdir + '/' + str(date) + '_polygon.jpg'

    cv2.imwrite(path, E.out_image)


if __name__ == "__main__":
    sys.exit(main())