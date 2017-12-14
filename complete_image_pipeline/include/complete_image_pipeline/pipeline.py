from duckietown_msgs.msg import Pixel, Segment, SegmentList  # @UnresolvedImport
from collections import OrderedDict
import copy

import cv2

from anti_instagram.AntiInstagram import AntiInstagram
import duckietown_utils as dtu
from easy_algo import get_easy_algo_db
from ground_projection import GroundProjection
from line_detector.visual_state_fancy_display import vs_fancy_display
from line_detector2.run_programmatically import FakeContext
import numpy as np


@dtu.contract(gp=GroundProjection)
def run_pipeline(image, gp, line_detector_name, image_prep_name):
    """ 
        Image: numpy (H,W,3) == BGR
        Returns a dictionary, res with the following fields:
            
            res['input_image']
    """
    res = OrderedDict()
    res['image_input'] = image
    algo_db = get_easy_algo_db()
    line_detector = algo_db.create_instance('line_detector', line_detector_name)
    image_prep = algo_db.create_instance('image_prep', image_prep_name)

    context = FakeContext()
    
#     segment_list = image_prep.process(context, image, line_detector, transform = None)
# 
#     res['segments_on_image_input'] = vs_fancy_display(image_prep.image_cv, segment_list)    
#     res['segments_on_image_resized'] = vs_fancy_display(image_prep.image_resized, segment_list)
    
    ai = AntiInstagram()
    ai.calculateTransform(res['image_input'])
    
    transform = ai.applyTransform 
    
    res['image_input_transformed'] = transform(res['image_input'])
    res['image_input_transformed_then_convertScaleAbs'] = cv2.convertScaleAbs(res['image_input_transformed'])

    segment_list2 = image_prep.process(context, image, line_detector, transform=transform)
    
    res['segments_on_image_input_transformed'] = \
        vs_fancy_display(image_prep.image_cv, segment_list2)
        
    res['segments_on_image_input_transformed_resized'] = \
        vs_fancy_display(image_prep.image_resized, segment_list2)

    res['grid'] = get_grid(image.shape[:2])
    
    res['grid_remapped'] = gp.rectify(res['grid'])
        
    res['image_input_rect'] = gp.rectify(res['image_input'])
    
#     res['difference between the two'] = res['image_input']*0.5 + res['image_input_rect']*0.5
#     
    segment_list2_rect = rectify_segments(gp, segment_list2)
    res['segments rectified on image rectified'] = \
        vs_fancy_display(res['image_input_rect'], segment_list2_rect)

    
    
    return res

def get_grid(shape, L=32):
    H, W = shape
    res = np.zeros((H,W,3),'uint8')
    for i in range(H):
        for j in range(W):
            cx = int(i/L) 
            cy = int(j/L)
            coli = (cx + cy) %2
            col = {0: (255,0,0), 1: (0,255,0)}
            res[i,j,:] = col[coli]
    return res

def rectify_segment(gp, s1):
    s2 = Segment()
    for i in range(2):
        # normalized coordinated
        nc = s1.pixels_normalized[i]
        # get pixel coordinates
        pixels = gp.gpc.vector2pixel(nc)
        uv = [pixels.u, pixels.v]
        # rectify
        pr = gp.rectify_point(uv)
        # recompute normalized
        t = Pixel()
        t.u = pr[0]
        t.v = pr[1]
        s = ' i %d  p = %s   pr = %s  normalized = %s' % (i, pr, nc, t)
        dtu.logger.info(s.replace('\n', ' '))
        s2.pixels_normalized[i] = gp.gpc.pixel2vector(t)
    return s2

    
def rectify_segments(gp, segment_list):
    S2 = SegmentList()
    
#     segment_list = copy.deepcopy(segment_list)
    
    for k, segment in enumerate(segment_list.segments):
        dtu.logger.info('%s - %s before' % (k, segment))
        
        s2 = rectify_segment(gp, segment)
            
        dtu.logger.debug('%s - %s after' % (k, s2))
        
        S2.segments.append(s2)
        
    return S2

    
