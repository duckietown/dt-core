from contracts.utils import check_isinstance
import numpy as np
from line_detector.visual_state import VisualState
from line_detector.line_detector_plot import drawLines
from duckietown_utils.jpg import make_images_grid


BLACK = (0,0,0)
BGR_RED = (0,0,255)
BGR_GREEN = (0,255,0)
BGR_WHITE = (255,255,255)
BGR_YELLOW = (0, 255,255)

def vs_fancy_display(vs):
    """
        vs: VisualState
        
        returns an image 
    """
    check_isinstance(vs, VisualState)
    im = vs.original_image
    
    # BGR
    img_white = np.copy(im)
    img_white.fill(0)
    drawLines(img_white, vs.white.lines, BGR_WHITE, p1_color=None, p2_color=None)
    
    img_yellow = np.copy(vs.original_image)
    img_yellow.fill(0)
    drawLines(img_yellow, vs.yellow.lines, BGR_YELLOW, p1_color=None, p2_color=None)
    
    img_red = np.copy(vs.original_image)
    img_red.fill(0)
    drawLines(img_red, vs.red.lines, BGR_RED, p1_color=None, p2_color=None)
    
    asgrid = make_images_grid([vs.original_image, img_white, img_yellow, img_red], 
                           cols=2, pad=10, bgcolor=[1, 1, 1])
    
    return asgrid 

