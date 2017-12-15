
from collections import OrderedDict

import cv2

from anti_instagram import AntiInstagram
import duckietown_utils as dtu
from easy_algo import get_easy_algo_db
from ground_projection import GroundProjection
from line_detector.visual_state_fancy_display import vs_fancy_display
from line_detector2.run_programmatically import FakeContext
import numpy as np
from lane_filter import FAMILY_LANE_FILTER
from line_detector.line_detector_interface import FAMILY_LINE_DETECTOR
from reprep.graphics.filter_scale import scale
from ground_projection.segment import rectify_segments
from duckietown_utils.matplotlib_utils import CreateImageFromPylab
from lane_filter.visualization import plot_phi_d_diagram_bgr


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

    grid = get_grid(image.shape[:2])
    if all_details:
        res['grid'] = grid

    res['grid_remapped'] = gp.rectify(grid)
        
    res['image_input_rect'] = gp.rectify(res['image_input'])
    
#     res['difference between the two'] = res['image_input']*0.5 + res['image_input_rect']*0.5
#     
    segment_list2_rect = rectify_segments(gp, segment_list2)
    res['segments rectified on image rectified'] = \
        vs_fancy_display(res['image_input_rect'], segment_list2_rect)

    # Project to ground
    sg = gp.find_ground_coordinates(segment_list2_rect)
    
    lane_filter.initialize()
    lane_filter.update(sg.segments)

    status = lane_filter.get_status()
    estimate = lane_filter.get_estimate()
    phi = estimate['phi']
    d = estimate['d']
    
    res['plot'] = plot_phi_d_diagram_bgr(lane_filter, phi=phi, d=d)
    
    
    belief_image = scale(-lane_filter.belief)
    
    res['belief (%s)' % status] = belief_image
#     
#         belief_img = bridge.cv2_to_imgmsg((255*self.filter.belief).astype('uint8'), "mono8")

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

    
