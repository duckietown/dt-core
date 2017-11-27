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
    pix_black = []
    pix_white = []
    pix_yellow = []
    pix_red = []

    # create instance with an image
    def __init__(self, input_image):
        self.image = input_image
        self.out_image = input_image
        print('created instance of estimateError!')

    def printVertices(self, polygon):

        # print black lines
        cv2.line(self.out_image,
                 (int(polygon.exterior.coords[0][0]), int(polygon.exterior.coords[0][1])),
                 (int(polygon.exterior.coords[1][0]), int(polygon.exterior.coords[1][1])),
                 (0, 0, 0))
        cv2.line(self.out_image,
                 (int(polygon.exterior.coords[1][0]), int(polygon.exterior.coords[1][1])),
                 (int(polygon.exterior.coords[2][0]), int(polygon.exterior.coords[2][1])),
                 (0, 0, 0))
        cv2.line(self.out_image,
                 (int(polygon.exterior.coords[2][0]), int(polygon.exterior.coords[2][1])),
                 (int(polygon.exterior.coords[3][0]), int(polygon.exterior.coords[3][1])),
                 (0, 0, 0))
        cv2.line(self.out_image,
                 (int(polygon.exterior.coords[3][0]), int(polygon.exterior.coords[3][1])),
                 (int(polygon.exterior.coords[0][0]), int(polygon.exterior.coords[0][1])),
                 (0, 0, 0))





    def GetErrorEstimation(self):
        height, width = self.image.shape[:2]
        print(self.image.shape[:2])
        # loop over image
        for j in range(width):
            for i in range(height):
                point = Point(j, i)
                # check if current pixel is within black polygon
                if(self.polygon_black.contains(point)):
                    # store indices to pixel
                    self.pix_black.append(self.image[i,j,:])

                # check if current pixel is within white polygon
                elif (self.polygon_white.contains(point)):
                    # store indices to pixel
                    self.pix_white.append(self.image[i,j,:])

                # check if current pixel is within yellow polygon
                elif (self.polygon_yellow.contains(point)):
                    # store indices to pixel
                    self.pix_yellow.append(self.image[i,j,:])

                # check if current pixel is within red polygon
                elif (self.polygon_red.contains(point)):
                    # store indices to pixel
                    self.pix_red.append(self.image[i,j,:])

        print('black len = ' + str(len(self.pix_black)))
        print('white len = ' + str(len(self.pix_white)))
        print('yellow len = ' + str(len(self.pix_yellow)))
        print('red len = ' + str(len(self.pix_red)))

        average_black = np.average(self.pix_black, axis=0)
        variance_black = np.var(self.pix_black, axis=0)
        average_white = np.average(self.pix_white, axis=0)
        variance_white = np.var(self.pix_white, axis=0)
        average_yellow = np.average(self.pix_yellow, axis=0)
        variance_yellow = np.var(self.pix_yellow, axis=0)
        average_red = np.average(self.pix_red, axis=0)
        variance_red = np.var(self.pix_red, axis=0)

        print('average black = ' + str(average_black))
        print('average white = ' + str(average_white))
        print('average yellow = ' + str(average_yellow))
        print('average red = ' + str(average_red))
        print('variance black = ' + str(variance_black))
        print('variance white = ' + str(variance_white))
        print('variance yellow = ' + str(variance_yellow))
        print('variance red = ' + str(variance_red))

        self.printVertices(self.polygon_black)
        self.printVertices(self.polygon_white)
        self.printVertices(self.polygon_yellow)
        self.printVertices(self.polygon_red)
        cv2.imshow('polygons', self.out_image)
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