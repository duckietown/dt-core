from error_estimation import estimateError
import cv2


# image mockup
img = cv2.imread('./test_img.png')



#  polygons for the colors
polygon_black = [(55,100),(180,320),(360,360),(350,30)]
polygon_white = [(1,1),(2,1),(1,2),(2,2)]
polygon_yellow = [(2,2),(3,2),(2,3),(3,3)]
polygon_red = [(3,3),(4,3),(3,4),(4,4)]

# create dictionary containing colors
polygons = {'black' : polygon_black, 'white' : polygon_white, 'yellow' : polygon_yellow, 'red' : polygon_red}


E = estimateError(img)
E.createPolygon(polygons)

E.GetErrorEstimation()