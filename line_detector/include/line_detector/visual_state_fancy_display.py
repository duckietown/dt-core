import cv2

from duckietown_msgs.msg import (Segment)  # @UnresolvedImport
import numpy as np

import duckietown_utils as dtu
AA = cv2.LINE_AA  # @UndefinedVariable

def vs_fancy_display(image_cv, segment_list, width=2):
    """
         
    """
    colors = {Segment.WHITE: dtu.ColorConstants.BGR_WHITE,
              Segment.RED: dtu.ColorConstants.BGR_RED,
              Segment.YELLOW: dtu.ColorConstants.BGR_YELLOW}
    
    ground = np.copy(image_cv)
    shape = ground.shape[:2]
    
    ground = ground / 8 + 80
     
    for segment in segment_list.segments:
        
        p1 = segment.pixels_normalized[0]
        p2 = segment.pixels_normalized[1]
        
        P1 = normalized_to_image(p1, shape)
        P2 = normalized_to_image(p2, shape) 
        
        paint = colors[segment.color]
        
        cv2.line(ground, P1, P2, paint, width, lineType=AA)
        
    return ground

def normalized_to_image(p,shape):
    x, y = p.x, p.y
    H, W = shape
    X = x * W
    Y = y * H
    return int(X), int(Y)

