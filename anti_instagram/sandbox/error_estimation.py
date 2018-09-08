#!/usr/bin/env python
import sys
import os
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import cv2
import numpy as np
import datetime

class estimateError:
    """ This class evaluates the quality of an image, which was transformed by a certain color transformation """

    # contains image to be analysed
    image = []
    out_image = []

    # hardcoded colors
    black = [60, 60, 60]
    red = [60, 60, 240]
    yellow = [50, 240, 240]
    white = [240, 240, 240]

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
        average_white = np.average(self.pix_white, axis=0)
        average_yellow = np.average(self.pix_yellow, axis=0)
        average_red = np.average(self.pix_red, axis=0)

        variance_black = np.var(self.pix_black, axis=0)
        variance_white = np.var(self.pix_white, axis=0)
        variance_yellow = np.var(self.pix_yellow, axis=0)
        variance_red = np.var(self.pix_red, axis=0)

        dist_black = np.linalg.norm(average_black - self.black)
        dist_white = np.linalg.norm(average_white - self.white)
        dist_yellow = np.linalg.norm(average_yellow - self.yellow)
        dist_red = np.linalg.norm(average_red - self.red)

        print(' ')
        print('summary:')
        print(' ')
        print(' ')
        print('BLACK:')
        print('average = ' + str(average_black))
        print('distance = ' + str(dist_black))
        print('variance = ' + str(variance_black))
        print(' ')
        print('WHITE')
        print('average = ' + str(average_white))
        print('distance = ' + str(dist_white))
        print('variance = ' + str(variance_white))
        print(' ')
        print('YELLOW')
        print('average = ' + str(average_yellow))
        print('distance = ' + str(dist_yellow))
        print('variance = ' + str(variance_yellow))
        print(' ')
        print('RED')
        print('average = ' + str(average_red))
        print('distance = ' + str(dist_red))
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


# the main function takes an image as argument, and calculates the estimated error
def main():
    # check number of arguments
    if len(sys.argv) != 3:
        print('This program takes an image file and an output directory as input.')
        sys.exit(2)

    # store input
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
    img = cv2.imread(file, cv2.IMREAD_UNCHANGED)

    """
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

    """
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
    E = estimateError(img)
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