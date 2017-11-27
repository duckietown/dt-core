from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import cv2
import numpy as np

class estimateError:
    """ This class evaluates the quality of an image, which was transformed by a certain color transformation """

    # contains image to be analysed
    image = []
    out_image = []

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
        self.out_image = input_image
        print('created instance of estimateError!')



    def GetErrorEstimation(self):
        height, width = self.image.shape[:2]
        # loop over image
        for j in range(width):
            for i in range(height):
                point = Point(i, j)
                # check if current pixel is within black polygon
                if(self.polygon_black.contains(point)):
                    #self.pix_black.append(self.image[i,j,:])
                    self.out_image[i,j] = (200, 200, 200)

                # check if current pixel is within white polygon
                elif (self.polygon_white.contains(point)):
                    #self.pix_white.append(self.image[i, j, :])
                    a = 1+1

                # check if current pixel is within yellow polygon
                elif (self.polygon_yellow.contains(point)):
                    #self.pix_yellow.append(self.image[i, j, :])
                    a = 1 + 1

                # check if current pixel is within red polygon
                elif (self.polygon_red.contains(point)):
                    #self.pix_red.append(self.image[i, j, :])
                    a = 1 + 1
        cv2.imshow('polygon', self.out_image)
        cv2.waitKey(0)


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