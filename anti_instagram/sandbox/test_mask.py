#!/usr/bin/env python
import cv2
import numpy as np
import sys
from geom import identifyLaneSurface
import datetime



if __name__=='__main__':
    fname = sys.argv[1]
    outdir = sys.argv[2]
    img = cv2.imread(fname)
    surf = identifyLaneSurface(img, use_hsv=False, grad_thresh=30)
    orig = img
    img = np.expand_dims(surf, -1) * img

    cv2.imshow('output', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # write the corrected image
    date = datetime.datetime.now().strftime("%H-%M-%S")
    path = outdir + '/' + str(date) + '_masked.jpg'

    cv2.imwrite(path, img)