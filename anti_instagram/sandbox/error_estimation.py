from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import cv2
import numpy as np

class estimateError:
    """ This class evaluates the quality of an image, which was transformed by a certain color transformation """

    # contains image to be analysed
    image = []

    # container for the polygons
    # TODO possibility to add more than one polygon per color
    polygon_black = Polygon()
    polygon_white = Polygon()
    polygon_yellow = Polygon()
    polygon_red = Polygon()

    # containers for pixels within polygons
    pix_black = np.empty((0,0))
    pix_white = np.empty((0,0))
    pix_yellow = np.empty((0,0))
    pix_red = np.empty((0,0))

    # create instance with an image
    def __init__(self, input_image):
        self.image = input_image
        print('created instance of estimateError!')

    def GetErrorEstimation(self, input_image, polygon):
        width, height = cv2.GetSize(input_image)
        arrayOfInPolygonPixels = np.empty((0,0))
        for i in range(width):
            for j in range(height):
                point = Point(i, j)
                if(polygon.contains(point)):
                    arrayOfInPolygonPixels.append(input_image[i,j,:])


    # this function takes a dictionary containing the polygons for the colors and creates the internal polygons:
    # input:
    # - polygons: dictionary {'black' : [], 'white' : [], 'yellow' : [], 'red' : []}
    def createPolygon(self, polygons):
        self.polygon_black = Polygon(polygons['black'])
        self.polygon_white = Polygon(polygons['white'])
        self.polygon_yellow = Polygon(polygons['yellow'])
        self.polygon_red = Polygon(polygons['red'])
        print('Created Polygons with the following vertices:')
        print('black: ' + str(self.polygon_black))
        print('white: ' + str(self.polygon_white))
        print('yellow: ' + str(self.polygon_yellow))
        print('red: ' + str(self.polygon_red))

    #def createPolygonByClick(self):







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