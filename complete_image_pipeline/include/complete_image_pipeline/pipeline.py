from collections import OrderedDict

import cv2
from nose.tools import assert_almost_equal

from anti_instagram import AntiInstagram
from duckietown_segmaps.maps import lane_straight_map, SegmentsMap, FRAME_AXLE
import duckietown_utils as dtu
from easy_algo import get_easy_algo_db
from geometry_msgs.msg import Point
from ground_projection import GroundProjection
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
from ground_projection.segment import rectify_segments
from lane_filter import FAMILY_LANE_FILTER
from line_detector.line_detector_interface import FAMILY_LINE_DETECTOR
from line_detector.visual_state_fancy_display import vs_fancy_display
from line_detector2.run_programmatically import FakeContext
import numpy as np

from .fuzzing import fuzzy_segment_list_image_space


@dtu.contract(gp=GroundProjection)
def run_pipeline(image, gp, line_detector_name, image_prep_name, lane_filter_name,
                 all_details=False):
    """ 
        Image: numpy (H,W,3) == BGR
        Returns a dictionary, res with the following fields:
            
            res['input_image']
    """
    
    
    res = OrderedDict()
    res['image_input'] = image
    algo_db = get_easy_algo_db()
    line_detector = algo_db.create_instance(FAMILY_LINE_DETECTOR, line_detector_name)
    lane_filter = algo_db.create_instance(FAMILY_LANE_FILTER, lane_filter_name)
    image_prep = algo_db.create_instance('image_prep', image_prep_name)

    context = FakeContext()
    
    if all_details:
        segment_list = image_prep.process(context, image, line_detector, transform = None)
     
        res['segments_on_image_input'] = vs_fancy_display(image_prep.image_cv, segment_list)    
        res['segments_on_image_resized'] = vs_fancy_display(image_prep.image_resized, segment_list)
    
    ai = AntiInstagram()
    ai.calculateTransform(image)
    
    transform = ai.applyTransform 
    
    transformed = transform(image)
    if all_details:
        res['image_input_transformed'] = transformed
        
    transformed_clipped = cv2.convertScaleAbs(transformed)
    
    res['image_input_transformed_then_convertScaleAbs'] = transformed_clipped

    segment_list2 = image_prep.process(context, transformed_clipped, line_detector, transform=transform)
    
    if all_details:
        res['segments_on_image_input_transformed'] = \
            vs_fancy_display(image_prep.image_cv, segment_list2)
        
    res['segments_on_image_input_transformed_resized'] = \
        vs_fancy_display(image_prep.image_resized, segment_list2)

    grid = get_grid(image.shape[:2])
    
    if all_details:
        res['grid'] = grid

        res['grid_remapped'] = gp.rectify(grid)
        
    rectified = gp.rectify(res['image_input'])
    
    if all_details:
        res['image_input_rect'] = rectified
    
#     res['difference between the two'] = res['image_input']*0.5 + res['image_input_rect']*0.5
#     
    segment_list2 = fuzzy_segment_list_image_space(segment_list2, n=100, intensity=0.0015)
#     segment_list2 = fuzzy_color(segment_list2)
    
    segment_list2_rect = rectify_segments(gp, segment_list2)
    res['segments rectified on image rectified'] = \
        vs_fancy_display(rectified, segment_list2_rect)

    # Project to ground
    sg = gp.find_ground_coordinates(segment_list2_rect)
    
    lane_filter.initialize()
    res['prior'] = lane_filter.get_plot_phi_d()
    
    lane_filter.update(sg.segments)

    res['belief'] = lane_filter.get_plot_phi_d()  
    res['segments reprojected on estimate'] = lane_filter.get_plot_plot_reprojected_bgr()
    
    lm = lane_straight_map()
    print lm
    
    gpg = gp.gpc # XXX
    
    est = lane_filter.get_estimate()
    camera_xy = np.array([0, -lane_filter.lanewidth/2+est['d'], 0])
    camera_theta = est['phi'] 
    
    # XXX
    gpg.rectified_input = True
    res['reprojected'] = plot_map(rectified, lm, gpg, camera_xy, camera_theta)

    return res

@dtu.contract(xyz='array[3]', cam_xyz='array[3]', cam_theta='float', returns='array[3]')
def camera_from_world(xyz, cam_xyz, cam_theta):
    """ Converts an xyz point in world coordinates, to camera coordinates """ 
    C = np.cos(-cam_theta)
    S = np.sin(-cam_theta)
    R = np.array([[C,-S,0,-cam_xyz[0]],
         [S,+C,0,-cam_xyz[1]],
         [0, 0,1,-cam_xyz[2]],
         [0,0,0,1]])
    a = np.array([xyz[0], xyz[1], xyz[2], 1])
#     dtu.logger.debug(R)
#     dtu.logger.debug(a)
    r = np.dot(R, a)
    return r[0:3]
         
    
@dtu.contract(sm=SegmentsMap, camera_xyz='array[3]', camera_theta='float', 
              gpg=GroundProjectionGeometry)
def plot_map(base, sm, gpg, camera_xyz, camera_theta):
    """
        base: already rectified image
    """
    image = base.copy()
    
    for segment in sm.segments:
        p1 = segment.points[0]
        p2 = segment.points[1]
        
        # If we are both in FRAME_AXLE
        if ( (sm.points[p1].id_frame == FRAME_AXLE) and 
             (sm.points[p2].id_frame == FRAME_AXLE)):
        
            # First, convert world to camera
            w1 = camera_from_world(sm.points[p1].coords, camera_xyz, camera_theta)
            w2 = camera_from_world(sm.points[p2].coords, camera_xyz, camera_theta)
            
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

def get_grid(shape, L=32, col= {0: (255,0,0), 1: (0,255,0)}):
    """ Creates a grid of given shape """
    H, W = shape
    res = np.zeros((H, W, 3), 'uint8')
    for i in range(H):
        for j in range(W):
            cx = int(i / L) 
            cy = int(j / L)
            coli = (cx + cy) % 2
            res[i,j,:] = col[coli]
    return res




def reproject_lane_on_image(image, phi, d):
    pass
