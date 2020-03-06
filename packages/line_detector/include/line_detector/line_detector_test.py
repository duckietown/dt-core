from . import LineDetector
from . import ColorRange
import cv2
import numpy as np

image = cv2.imread('test_image.jpg')

cv2.imshow('Image', image)

ld = LineDetector(canny_thresholds= [80, 200], canny_aperture_size=3, dilation_kernel_size=3,
                  hough_threshold= 2, hough_min_line_length= 3, hough_max_line_gap= 1)
ld.setImage(image)

cv2.imshow('Canny', ld.canny_edges)

red_dict = {'low_1': [0,140,100],
    'high_1': [15,255,255],
    'low_2': [165,140,100],
    'high_2': [180,255,255]}

white_dict = {'low':  [0,0,150],
              'high':  [180,60,255]}

yellow_dict = {'low' :[25, 140, 100],
               'high': [45, 255, 255]}

cr_red = ColorRange.fromDict(red_dict)
cr_white = ColorRange.fromDict(white_dict)
cr_yellow = ColorRange.fromDict(yellow_dict)

red_region, red_edges = ld.colorFilter(cr_red)
# cv2.imshow('Red region', red_region)
# cv2.imshow('Red Canny', red_edges)

white_region, white_edges = ld.colorFilter(cr_white)
# cv2.imshow('White region', white_region)
cv2.imshow('White Canny', white_edges)
#
# yellow_region, yellow_edges = ld.colorFilter(cr_yellow)
# cv2.imshow('Yellow region', yellow_region)
# cv2.imshow('Yellow Canny', yellow_edges)

ld.houghLine(white_edges)
detections = ld.detectLines(cr_white)
detection_centers = np.zeros_like(white_region)
for d in detections.centers:
    try:
        detection_centers[d[1]-5:d[1]+5, d[0]-5:d[0]+5] = 255
    except:
        pass
cv2.imshow('Centers', detection_centers)

detection_normals = np.zeros_like(white_region)
for i in range(len(detections.normals)):
    d = detections.centers[i]
    n = detections.normals[i]
    cv2.line(detection_normals, tuple(d), tuple((d+10*n).astype(int)), color=255)
    detection_normals[d[1] - 2:d[1] + 2, d[0] - 2:d[0] + 2] = 255

cv2.imshow('Normals', detection_normals)

cv2.waitKey(0)
cv2.destroyAllWindows()