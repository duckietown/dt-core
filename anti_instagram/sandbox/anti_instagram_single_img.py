#!/usr/bin/env python
import sys
import os
import cv2
import datetime
from anti_instagram.AntiInstagram import *



# check number of arguments 
if len(sys.argv) !=3:
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


#create instance of AntiInstagram
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


