from error_estimation import estimateError
import cv2
import time
import numpy as np

# image mockup
img = cv2.imread('./pic3_smaller.jpg')


#  polygons for pic3_smaller.jpg
polygon_black = [(280,570),(220,760),(870,750),(810,580)]
polygon_white = [(900,520),(1000,640),(1060,620),(970,515)]
polygon_yellow = [(234,430),(190,485),(230,490),(270,430)]
polygon_red = [(285,435),(250,490),(830,480),(800,437)]
"""

# polygons of color_test.jpg
polygon_black = [(25,30),(25,200),(230,200),(230,30)]
polygon_white = [(370,90),(430,230),(600,160),(550,29)]
polygon_yellow = [(450,270),(450,370),(570,370),(570,270)]
polygon_red = [(33,316),(45,380),(312,337),(300,275)]

# polygons of color_test.jpg without blend
polygon_black = [(25,30),(25,200),(200,170),(230,30)]
polygon_white = [(370,90),(430,230),(600,160),(530,55)]
polygon_yellow = [(450,270),(477,345),(570,370),(570,270)]
polygon_red = [(33,316),(45,380),(277,320),(300,275)]
"""
# create dictionary containing colors
polygons = {'black' : polygon_black, 'white' : polygon_white, 'yellow' : polygon_yellow, 'red' : polygon_red}


E = estimateError(img)
E.createPolygon(polygons)

start = time.time()
E.GetErrorEstimation()
end = time.time()
print('GetErrorEstimation took ' + str(end-start) + 'sec.')
