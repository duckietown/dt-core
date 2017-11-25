from error_estimation import estimateError

# image mockup
arr = [(0,0),(1,0),(0,1),(1,1)]

#  polygons for the colors
polygon_black = [(0,0),(1,0),(0,1),(1,1)]
polygon_white = [(1,1),(2,1),(1,2),(2,2)]
polygon_yellow = [(2,2),(3,2),(2,3),(3,3)]
polygon_red = [(3,3),(4,3),(3,4),(4,4)]

# create dictionary containing colors
polygons = {'black' : polygon_black, 'white' : polygon_white, 'yellow' : polygon_yellow, 'red' : polygon_red}


E = estimateError(arr)
E.createPolygon(polygons)
