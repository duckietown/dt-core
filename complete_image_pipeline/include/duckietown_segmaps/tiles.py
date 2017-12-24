from duckietown_msgs.msg import Segment
from numpy.testing.utils import assert_almost_equal

import duckietown_utils as dtu
from .maps import SegmentsMap, FRAME_TILE, SegMapPoint, SegMapFace, SegMapSegment

YELLOW = dtu.ColorConstants.STR_YELLOW
WHITE = dtu.ColorConstants.STR_WHITE
BLACK = dtu.ColorConstants.STR_BLACK
GRAY = dtu.ColorConstants.STR_GRAY
RED = dtu.ColorConstants.STR_RED


    
@dtu.contract(returns=SegmentsMap)
def empty_tile(tile_size, tile_spacing):
    constants = {}
    constants['tile_size'] = tile_size
    constants['tile_spacing'] = tile_spacing
    points = {}
    segments = []
    faces = []
    
    add_tile(points, faces, segments, tile_size, tile_spacing)
    
    data = dict(points=points, segments=segments, faces=faces, constants=constants)

    return SegmentsMap(**data)

@dtu.contract(returns=SegmentsMap)
def get_map_straight_lane(tile_size, width_yellow, width_white, tile_spacing,
                          gap_len,
                          dash_len, width_red):
    # from inner yellow to inner white
    constants = {}
    constants['tile_size'] = tile_size
    constants['width_yellow'] = width_yellow
    constants['width_white'] = width_white
    constants['dash_len'] = dash_len
    constants['gap_len'] = gap_len
    constants['tile_spacing'] = gap_len

    lane_width = L = (tile_size - 2*width_white - width_yellow) / 2
    constants['lane_width'] = lane_width
    msg = """
        width_yellow: %s   
        width_white: %s
        tile "size": %s  (outer white to outer white) 
        lane width: %s  (inner yellow to inner white)
    """ % (width_yellow, width_white, tile_size, lane_width)
    dtu.logger.info(msg)
    y1 = + width_yellow/2 + L + width_white
    y2 = + width_yellow/2 + L
    y3 = + width_yellow/2
    y4 = - width_yellow/2
    y5 = - width_yellow/2 - L
    y6 = - width_yellow/2 - L - width_white  
    
    assert_almost_equal(width_white + L + width_yellow + L + width_white, tile_size) 
    
    extra = (tile_spacing - tile_size)/2
#     tile_inter = (tile_spacing - tile_size)/2
    
    # horizon = 2 cells
#     num_cells_horizon  = 1
#     D = (num_cells_horizon + 0.5)* tile_size

    FRAME = FRAME_TILE
    
    S = -tile_size/2 - extra
    D = tile_size/2 + extra
    points = {}
    segments = []
    faces = []
    
    points['p1'] = SegMapPoint(id_frame=FRAME, coords=[S, y1, 0])
    points['q1'] = SegMapPoint(id_frame=FRAME, coords=[D, y1, 0])
    points['p2'] = SegMapPoint(id_frame=FRAME, coords=[S, y2, 0])
    points['q2'] = SegMapPoint(id_frame=FRAME, coords=[D, y2, 0])
    points['p3'] = SegMapPoint(id_frame=FRAME, coords=[S, y3, 0])
    points['q3'] = SegMapPoint(id_frame=FRAME, coords=[D, y3, 0])
    points['p4'] = SegMapPoint(id_frame=FRAME, coords=[S, y4, 0])
    points['q4'] = SegMapPoint(id_frame=FRAME, coords=[D, y4, 0])
    points['p5'] = SegMapPoint(id_frame=FRAME, coords=[S, y5, 0])
    points['q5'] = SegMapPoint(id_frame=FRAME, coords=[D, y5, 0])
    points['p6'] = SegMapPoint(id_frame=FRAME, coords=[S, y6, 0])
    points['q6'] = SegMapPoint(id_frame=FRAME, coords=[D, y6, 0])
    
   
    add_tile(points, faces, segments, tile_size, tile_spacing)
        
    def add_dash(x):
        s = len(points)
        pre = 'dash%s_' % s
        points[pre+'t0'] = SegMapPoint(id_frame=FRAME, coords=[x, y3, 0])
        points[pre+'t1'] = SegMapPoint(id_frame=FRAME, coords=[x, y4, 0])
        points[pre+'t2'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y4, 0])
        points[pre+'t3'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y3, 0])
        faces.append(SegMapFace(color=YELLOW, 
                                points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    
    ngaps = (tile_size) / (gap_len + dash_len) 
    for i in range(int(ngaps)):
        x = i * (gap_len + dash_len) - tile_size/2
        add_dash(x)
    
    segments.append(SegMapSegment(color=Segment.WHITE, points=['q1','p1']))
    segments.append(SegMapSegment(color=Segment.WHITE, points=['p2','q2']))
    segments.append(SegMapSegment(color=Segment.YELLOW, points=['q3','p3']))
    segments.append(SegMapSegment(color=Segment.YELLOW, points=['p4','q4']))
    segments.append(SegMapSegment(color=Segment.WHITE, points=['q5','p5']))
    segments.append(SegMapSegment(color=Segment.WHITE, points=['p6','q6']))
    
    faces.append(SegMapFace(color=WHITE, points=['p1', 'q1', 'q2', 'p2']))
    faces.append(SegMapFace(color=WHITE, points=['p5', 'q5', 'q6', 'p6']))

    if width_red is not None: 
        x1 = tile_size/2 - width_red
        y2 = -width_yellow/2
        y1 = -tile_size/2+width_white
        x2 = tile_size/2 
        
        _add_rect(points, faces, segments, x1, y1, x2, y2, FRAME, RED, 
                  use_sides_for_loc=[Segment.RED,None,Segment.RED,None])
        constants['width_red'] = width_red
        
    data = dict(points=points, segments=segments, faces=faces, constants=constants)

    return SegmentsMap(**data)

@dtu.contract(points='dict', faces='list', id_frame='str', color='str',
              use_sides_for_loc='list[4](None|int)')
def _add_rect(points, faces, segments, x1, y1, x2, y2, id_frame, color,
              use_sides_for_loc):
    assert x2 > x1
    assert y2 > y1
    s = len(points)
    
    pre = '%s_' % s
    names=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']
    points[names[0]] = SegMapPoint(id_frame=id_frame, coords=[x1, y1, 0])
    points[names[1]] = SegMapPoint(id_frame=id_frame, coords=[x1, y2, 0])
    points[names[2]] = SegMapPoint(id_frame=id_frame, coords=[x2, y2, 0])
    points[names[3]] = SegMapPoint(id_frame=id_frame, coords=[x2, y1, 0])
    faces.append(SegMapFace(color=color,points=names))
    
    for i, c in enumerate(use_sides_for_loc):
        if c is not None:
            # note order is important because it should be counterclockwise
            p0 = names[(i+1)%4]
            p1 = names[i]
            segments.append(SegMapSegment(color=c, points=[p0,p1]))


def add_tile(points, faces, segments, tile_size, tile_spacing):
    """ Add tile bg at 0,0 """
    extra = (tile_spacing - tile_size)/2
    x1 = -tile_size/2 - extra
    x2 = +tile_size/2 + extra
    
    c = BLACK
    # to debug:
    # c = GRAY
    assert_almost_equal(x2-x1, tile_spacing)
    y1 = -tile_size/2 - extra
    y2 = +tile_size/2 + extra
    _add_rect(points, faces, segments, x1, y1, x2, y2, FRAME_TILE, c, 
              use_sides_for_loc=[None,None,None,None])

    x1 = -tile_size/2 
    x2 = +tile_size/2 
    y1 = -tile_size/2 
    y2 = +tile_size/2 
    _add_rect(points, faces, segments, x1, y1, x2, y2, FRAME_TILE, BLACK, 
              use_sides_for_loc=[None,None,None,None])

      
