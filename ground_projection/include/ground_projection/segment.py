from duckietown_msgs.msg import Pixel, Segment, SegmentList
import duckietown_utils as dtu
from ground_projection.ground_projection_geometry import GroundProjectionGeometry

@dtu.contract(gpg=GroundProjectionGeometry, s1=Segment, returns=Segment)
def rectify_segment(gpg, s1):
    s2 = Segment()
    for i in range(2):
        # normalized coordinates
        nc = s1.pixels_normalized[i]
        # get pixel coordinates
        pixels = gpg.vector2pixel(nc)
        uv = [pixels.u, pixels.v]
        # rectify
        pr = gpg.rectify_point(uv)
        # recompute normalized coordinates
        t = Pixel(pr[0], pr[1])
        s2.pixels_normalized[i] = gpg.pixel2vector(t)
    s2.color = s1.color
    return s2

@dtu.contract(gpg=GroundProjectionGeometry, segment_list=SegmentList,
              returns=SegmentList)
def rectify_segments(gpg, segment_list):
    S2 = SegmentList()
    
    for segment in segment_list.segments: 
        s2 = rectify_segment(gpg, segment) 
        S2.segments.append(s2)
    return S2


