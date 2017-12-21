from duckietown_msgs.msg import Pixel, Segment, SegmentList

def rectify_segment(gp, s1):
    s2 = Segment()
    for i in range(2):
        # normalized coordinates
        nc = s1.pixels_normalized[i]
        # get pixel coordinates
        pixels = gp.gpc.vector2pixel(nc)
        uv = [pixels.u, pixels.v]
        # rectify
        pr = gp.rectify_point(uv)
        # recompute normalized coordinates
        t = Pixel()
        t.u = pr[0]
        t.v = pr[1]
        s2.pixels_normalized[i] = gp.gpc.pixel2vector(t)
    s2.color = s1.color
    return s2


def rectify_segments(gp, segment_list):
    S2 = SegmentList()
    
    for segment in segment_list.segments: 
        s2 = rectify_segment(gp, segment) 
        S2.segments.append(s2)
        
    return S2


