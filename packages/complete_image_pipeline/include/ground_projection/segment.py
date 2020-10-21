from duckietown_msgs.msg import Pixel, Segment, SegmentList
from image_processing.ground_projection_geometry import GroundProjectionGeometry


def rectify_segment(gpg: GroundProjectionGeometry, s1: Segment) -> Segment:
    pixels_normalized = []

    for i in (0, 1):
        # normalized coordinates
        nc = s1.pixels_normalized[i]
        # get pixel coordinates
        pixels = gpg.vector2pixel(nc)
        uv = (pixels.u, pixels.v)
        # rectify
        pr = gpg.rectify_point(uv)
        # recompute normalized coordinates
        t = Pixel(pr[0], pr[1])
        v = gpg.pixel2vector(t)
        pixels_normalized.append(v)

    s2 = Segment(color=s1.color, pixels_normalized=pixels_normalized)
    return s2


def rectify_segments(gpg: GroundProjectionGeometry, segment_list: SegmentList) -> SegmentList:
    res = []

    for segment in segment_list.segments:
        s2 = rectify_segment(gpg, segment)
        res.append(s2)

    return SegmentList(segments=res)
