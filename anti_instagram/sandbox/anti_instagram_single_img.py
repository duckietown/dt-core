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


def GetListOfPixelsInPolygon(self, input_image, polygon):
    width, height = cv2.GetSize(input_image)
    arrayOfInPolygonPixels = np.empty((0, 0))
    for i in range(width):
        for j in range(height):
            point = Point(i, j)
            if (polygon.contains(point)):
                arrayOfInPolygonPixels.append(input_image[i, j, :])
    return arrayOfInPolygonPixels


yellowPolygon = Polygon([(356, 1298), (445, 1322), (286, 1540), (185, 1502)])
redPolygon = Polygon([(681, 1045), (595, 1178), (2027, 1160), (1937, 1047)])
whitePolygon = Polygon(
    [(1988, 1040), (2130, 1216), (2284, 1194), (2152, 1050)])
blackPolygon = Polygon([(1002, 744), (1484, 732), (1616,  948), (880, 918)])

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

#lets do the masking!
#surf = identifyLaneSurface(input_img, use_hsv=False, grad_thresh=20)
#input_img = np.expand_dims(surf, -1) * input_img
#cv2.imshow('mask', input_img)
#cv2.waitKey(0)

# create instance of AntiInstagram
ai = AntiInstagram()

ai.calculateTransform(input_img)
print('Transform Calculation completed!')

corrected_img = ai.applyTransform(input_img)
print('Transform applied!')


# write the corrected image
date = datetime.datetime.now().strftime("%H-%M-%S")
path = outdir + '/' + str(date) + '_corrected.jpg'

cv2.imwrite(path, corrected_img)


print('calculated scale: ' + str(ai.scale))
print('calculated shift: ' + str(ai.shift))
print('calculated health: ' + str(ai.health))
