from collections import namedtuple

import yaml
 
import duckietown_utils as dtu  
import numpy as np


SegMapPoint = namedtuple('SegMapPoint', 'id_frame coords') 
SegMapSegment = namedtuple('SegMapSegment', 'points color')
SegMapFace = namedtuple('SegMapFace', 'points color')

FRAME_AXLE = 'axle'
FRAME_TILE = 'tile'

class SegmentsMap(object):
    
    @dtu.contract(points=dict, segments=list, faces=list)
    def __init__(self, points, segments, faces):
        self.points = {}
        for k, p in points.items():
            self.points[k] = SegMapPoint(id_frame = p[0], coords= np.array(p[1]))
        
        self.segments = []
        for s in segments:
            S = SegMapSegment(points=s['points'], color=s['color'])
            self.segments.append(S)

        self.faces = []
        for f in faces:
            F = SegMapFace(points=f['points'], color=f['color'])
            self.faces.append(F)
        
        self.validate()
        
    def validate(self):
        for S in self.segments:
            for p in S.points:
                if not p in self.points:
                    msg = 'Invalid point %r' % p
                    raise ValueError(msg)
        for F in self.faces:
            for p in F.points:
                if not p in self.points:
                    msg = 'Invalid point %r' % p
                    raise ValueError(msg)
                
# 
#     @dtu.contract(id_point=str, gpg=GroundProjectionGeometry, returns=Pixel)
#     def get_screen_coordinates(self, id_point, gpg):
#         """ Returns the pixel coordinates for the original image """
#         if not id_point in self.points:
#             msg = 'Could not find point %r.' % id_point
#             raise ValueError(msg)
#         p = self.points[id_point]
#         if p.id_frame == FRAME_AXLE:
#             q = Point()
#             q.x = p.coords[0]
#             q.y = p.coords[1]
#             q.z = p.coords[2] 
# 
#             pixel = gpg.ground2pixel(q)
# 
#             dtu.logger.debug('q = %s  pixel = %s ' % (q, pixel))
#             
#             return pixel
#         else:
#             msg = 'Unknown axis %r' % p.id_frame
#             raise ValueError(msg)
#         
def lane_straight_map():
    m = yaml.load(maps['lane_straight'])
    return SegmentsMap(**m) 
    

maps = {'lane_straight': """
points:
     p1: [axle, [0, 0.2921, 0]]
     q1: [axle, [1.2192, 0.2921, 0]]
     p2: [axle, [0, 0.2413, 0]]
     q2: [axle, [1.2192, 0.2413, 0]]
     p3: [axle, [0, 0.0127, 0]]
     q3: [axle, [1.2192, 0.0127, 0]]
     p4: [axle, [0, -0.0127, 0]]
     q4: [axle, [1.2192, -0.0127, 0]]
     p5: [axle, [0, -0.2413, 0]]
     q5: [axle, [1.2192, -0.2413, 0]]
     p6: [axle, [0, -0.2921, 0]]
     q6: [axle, [1.2192, -0.2921, 0]]
segments:
 - points: [p1, q1]
   color: white
 - points: [p2, q2]
   color: white
 - points: [p3, q3]
   color: yellow
 - points: [p4, q4]
   color: yellow
 - points: [p5, q5]
   color: white
 - points: [p6, q6]
   color: white
"""}

