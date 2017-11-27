from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import cv2
import numpy as np

class estimateError:
    def __init__(self):
        pass
    def GetErrorEstimation(self, input_image, polygon):
        width, height = cv2.GetSize(input_image)
        arrayOfInPolygonPixels = np.empty((0,0))
        for i in range(width):
            for j in range(height):
                point = Point(i, j)
                if(polygon.contains(point)):
                    arrayOfInPolygonPixels.append(input_image[i,j,:])







'''

import cv2
import numpy as np

# original image
# -1 loads as-is so if it will be 3 or 4 channel as the original
image = cv2.imread('image.png', -1)
# mask defaulting to black for 3-channel and transparent for 4-channel
# (of course replace corners with yours)
mask = np.zeros(image.shape, dtype=np.uint8)
roi_corners_red = np.array([[(10,10), (300,300), (10,300)]], dtype=np.int32)
# fill the ROI so it doesn't get wiped out when the mask is applied
channel_count = image.shape[2]  # i.e. 3 or 4 depending on your image
ignore_mask_color = (255,)*channel_count
cv2.fillPoly(mask, roi_corners, ignore_mask_color)
# from Masterfool: use cv2.fillConvexPoly if you know it's convex

# apply the mask
masked_image = cv2.bitwise_and(image, mask)

# save the result
cv2.imwrite('image_masked.png', masked_image)

'''