import cv2
from nose.tools import assert_almost_equal

from duckietown_segmaps.maps import SegmentsMap, FRAME_AXLE
import duckietown_utils as dtu
from geometry_msgs.msg import Point
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
import numpy as np
from line_detector.visual_state_fancy_display import BGR_YELLOW, BGR_WHITE,\
    BGR_BLACK, BGR_BLUE
 
def rotate(l, n):
    return l[n:] + l[:n]

@dtu.contract(coords0='list(seq)', x_frustum='float,>0')
def get_points_rect_coords(coords0, x_frustum):
    """
        coords0 : list
    """
    n = len(coords0)
    points = range(n)
    
    """ Returns a list of world coordinates """
    def is_behind(i):
        w = coords0[i]
        behind = w[0] < x_frustum
        return behind
    
    # first check that not all are behind
    b = [is_behind(_) for _ in range(n)]
    
    if all(b):
        return []
    
    # there is at least one not behind
    while is_behind(points[0]):
        points = rotate(points, 1)
    
    assert not is_behind(points[0])
    # do the first one
    points.append(points[0])
    # now walk each point
    coords = []
    for i, id_point in enumerate(points):
        w = coords0[id_point]
        if i == 0:
            # assume the first one is not behind
            assert not is_behind(id_point)
#             print('%d: put first of course' % i)
            coords.append(w)
        else:
            previous = coords0[points[i-1]]

            if is_behind(id_point):
                if is_behind(points[i-1]):
                    # both outside just ignore
#                     print('%d: this and previous outside: add 1' % i)
                    pass
                else:
                    # this outside, previous was not
                    w_cut, _,  = clip_to_frustum(w, previous, x_frustum, noswap=True)
                    coords.append(w_cut)
                    
#                     print('%d: this outside, previous was not: add 1' % i)
            else: # not behind
                if is_behind(points[i-1]):
                    # previous outside, this not
                    previous_cut, w = clip_to_frustum(previous, w, x_frustum, noswap=True)
                    coords.append(previous_cut)
                    coords.append(w) 
#                     print('%d: previous outside, this not: add 2' % i)
                else:
                    # previous inside, this also: normal
#                     print('%d: previous and this inside: add 1' % i)
                    coords.append(w)
    return coords

def bgr_color_from_string(s):
    d = {
        'yellow': BGR_YELLOW,
        'white': BGR_WHITE,
        'black': BGR_BLACK,
        'blue': BGR_BLUE,
    }
    return d[s]


def paint_polygon_world(base, coords, gpg, color, x_frustum):
    coords_inside = get_points_rect_coords(coords, x_frustum=x_frustum)
    if not coords_inside:
        # all outside
        return
    
    # pixel coords
    def pixel_from_world(c):
        p = gpg.ground2pixel(Point(c[0],c[1],c[2]))
        return (int(p.u), int(p.v))
    
    cv_points = np.array(map(pixel_from_world, coords_inside), dtype='int32')
    
    cv2.fillPoly(base, [cv_points], color)
    
def plot_ground_sky(base, gpg, color_ground, color_sky):
    x = +100
    p = gpg.ground2pixel(Point(x,0,0))
    v = int(p.v)
    for i in range(3):
        base[v:, :, i] = color_ground[i]
    for i in range(3):
        base[:v, :, i] = color_sky[i]
    
@dtu.contract(sm=SegmentsMap, #camera_xyz='array[3]', camera_theta='float', 
              gpg=GroundProjectionGeometry)
def plot_map(base0, sm, gpg, do_ground=True, do_faces=True, do_segments=True): #, camera_xyz, camera_theta):
    """
        base: already rectified image 
        
        sm= SegmentsMap in frame FRAME_AXLE
    """
    image = base0.copy()
    x_frustum = +0.05
    if do_ground:
        color_ground = (30,10,22)
        color_sky = (244, 134, 66)
        plot_ground_sky(image, gpg, color_ground, color_sky)
        
    if do_faces:
        for face in sm.faces:
            # list of arrays with cut stuff
            coords = [sm.points[_].coords for _ in face.points]
            color = bgr_color_from_string(face.color)
            paint_polygon_world(image, coords, gpg, color, x_frustum)
    
    if do_segments:
        for segment in sm.segments:
            p1 = segment.points[0]
            p2 = segment.points[1]
            
            # If we are both in FRAME_AXLE
            if ( (sm.points[p1].id_frame != FRAME_AXLE) or 
                 (sm.points[p2].id_frame != FRAME_AXLE)):
                msg = "Cannot deal with points not in frame FRAME_AXLE"
                raise NotImplementedError(msg)
            
            w1 = sm.points[p1].coords
            w2 = sm.points[p2].coords
            
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
    
    return image
    
@dtu.contract(w1='array[w]', w2='array[3]', x_frustum='float')
def clip_to_frustum(w1, w2, x_frustum, noswap=False):
    """ Assumes that w1 is outside. Returns w1_cut, w2 """    
    if not noswap:
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