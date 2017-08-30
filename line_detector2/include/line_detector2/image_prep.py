import cv2

from duckietown_msgs.msg import (Segment, SegmentList)  # @UnresolvedImport
from line_detector2.ldn import toSegmentMsg
import numpy as np


class ImagePrep():
    
    def __init__(self, shape, top_cutoff):
        self.shape = shape
        self.top_cutoff = top_cutoff
    
    def process(self, context, image_cv, line_detector, transform):
        """ Returns SegmentList """
        
        self.image_cv = image_cv
        with context.phase('resizing'):
            # Resize and crop image
            h0, w0 = image_cv.shape[0:2]
            h1, w1 = self.shape
            
            if (h0,w0) != (h1,w1):
                # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
                self.image_resized = cv2.resize(image_cv, (w1,h1), interpolation=cv2.INTER_NEAREST)
            else:
                self.image_resized = image_cv
            self.image_cut = self.image_resized[self.top_cutoff:,:,:]
            
            
        with context.phase('correcting'):
            # apply color correction: AntiInstagram
            if transform is not None:
                _ = transform(self.image_cut)
                # XXX
                self.image_corrected = cv2.convertScaleAbs(_)
            else:
                self.image_corrected = self.image_cut
 
        with context.phase('detection'):
            # Set the image to be detected
            
            line_detector.setImage(self.image_corrected)
    
            # Detect lines and normals    
            white = line_detector.detectLines('white')
            yellow = line_detector.detectLines('yellow')
            red = line_detector.detectLines('red')
            segment_list = get_segment_list_normalized(self.top_cutoff, self.shape, white, yellow, red)
            
        # SegmentList constructor
        return segment_list
    
#             self.intermittent_log('# segments: white %3d yellow %3d red %3d' % (len(white.lines),
#                     len(yellow.lines), len(red.lines)))
#             


def get_segment_list_normalized(top_cutoff, shape, white, yellow, red):
    segmentList = SegmentList() 
    
    # Convert to normalized pixel coordinates, and add segments to segmentList
    s0, s1 = shape
    arr_cutoff = np.array((0, top_cutoff, 0, top_cutoff))
    arr_ratio = np.array((1./ s1, 1./ s0, 1./ s1, 1./ s0))
    
    if len(white.lines) > 0:
        lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
        segmentList.segments.extend(toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
    
    if len(yellow.lines) > 0:
        lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
        segmentList.segments.extend(toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
    
    if len(red.lines) > 0:
        lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
        segmentList.segments.extend(toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))
        
    return segmentList
            
            