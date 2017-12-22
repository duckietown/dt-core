import cv2
from nose.tools import assert_almost_equal

from duckietown_segmaps.maps import SegmentsMap, FRAME_AXLE
import duckietown_utils as dtu
from geometry_msgs.msg import Point
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
import numpy as np

# 
# @dtu.contract(xyz='array[3]', cam_xyz='array[3]', cam_theta='float', returns='array[3]')
# def camera_from_world(xyz, cam_xyz, cam_theta):
#     """ Converts an xyz point in world coordinates, to camera coordinates """ 
#     C = np.cos(-cam_theta)
#     S = np.sin(-cam_theta)
#     R = np.array([[C,-S,0,-cam_xyz[0]],
#          [S,+C,0,-cam_xyz[1]],
#          [0, 0,1,-cam_xyz[2]],
#          [0,0,0,1]])
#     a = np.array([xyz[0], xyz[1], xyz[2], 1])
# #     dtu.logger.debug(R)
# #     dtu.logger.debug(a)
#     r = np.dot(R, a)
#     return r[0:3]
#          
    
@dtu.contract(sm=SegmentsMap, #camera_xyz='array[3]', camera_theta='float', 
              gpg=GroundProjectionGeometry)
def plot_map(base, sm, gpg): #, camera_xyz, camera_theta):
    """
        base: already rectified image 
        
        sm= SegmentsMap in frame FRAME_AXLE
    """
    image = base.copy()
    
    for segment in sm.segments:
        p1 = segment.points[0]
        p2 = segment.points[1]
        
        # If we are both in FRAME_AXLE
        if ( (sm.points[p1].id_frame == FRAME_AXLE) and 
             (sm.points[p2].id_frame == FRAME_AXLE)):
        
            w1 = sm.points[p1].coords
            w2 = sm.points[p2].coords
            x_frustum = +0.05
            w1_behind = w1[0] < x_frustum
            w2_behind = w2[0] < x_frustum
            
            if w1_behind and w2_behind:
                # They are both behind the camera: we do not draw them
                dtu.logger.debug('Will not draw %s %s' % (w1,w2))
                continue
            elif not w1_behind and not w2_behind:
                dtu.logger.debug('Points are ok')
                pass
            else:
                dtu.logger.debug('Segment needs clipping')
                w1, w2 = clip_to_frustum(w1, w2, x_frustum)
             
            uv1 = gpg.ground2pixel(Point(w1[0],w1[1],w1[2]))
            uv2 = gpg.ground2pixel(Point(w2[0],w2[1],w2[2]))

            width = 2
    #         BGR_WHITE = (255,255,255)
            paint = BGR_BLUE = (255,0,0)
    #         paint = BGR_WHITE
            ti = lambda a,b: (int(np.round(a)), int(np.round(b)))
            
            cv2.line(image, ti(uv1.u, uv1.v), ti(uv2.u, uv2.v), paint, width)
            
        else:
            msg = "Cannot deal with points not in frame FRAME_AXLE"
            raise NotImplementedError(msg)

        gpg.rectified_input = True

    return image
    
@dtu.contract(w1='array[w]', w2='array[3]', x_frustum='float')
def clip_to_frustum(w1, w2, x_frustum):    
    if w1[0] > x_frustum:
        return clip_to_frustum(w2, w1, x_frustum)
    assert w1[0] <= x_frustum
    assert w2[0] > x_frustum
    
    # n*p + d = 0
    n = np.array([1,0,0])
    d = -x_frustum
    
    direction = w2-w1
    
    # intersection = w2 + alpha * direction
    # n* (w2 + alpha * dir) + d = 0
    # (n*w2) + alpha (n*dir) + d = 0
    #   alpha = (-d-(n*w2))/(n*dir)
    alpha = (- d - np.dot(n, w2)) / (np.dot(n, direction))
    intersection = w2 + alpha * direction
    
    assert_almost_equal(intersection[0], x_frustum)
    
    w1_ = intersection
    return w1_, w2