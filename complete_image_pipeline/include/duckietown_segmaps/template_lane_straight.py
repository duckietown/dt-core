from numpy.testing.utils import assert_almost_equal

import duckietown_utils as dtu
import numpy as np

from .map_localization_template import LocalizationTemplate
from .maps import SegmentsMap, FRAME_TILE, SegMapPoint, SegMapFace, SegMapSegment
import warnings


class TemplateStraightLane(LocalizationTemplate):
    
    DATATYPE_COORDS = np.dtype([('phi', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_size, 
                       width_yellow, 
                       width_white,
                       tile_spacing):
        
        self.map = get_map_straight_lane(tile_size, width_yellow, width_white, tile_spacing)
        self.lane_width = (tile_size - 2*width_white - width_yellow) / 2
        self.dt = TemplateStraightLane.DATATYPE_COORDS
        self.offset = width_yellow/2 + self.lane_width/2
        
    @dtu.contract(returns=SegmentsMap)
    def get_map(self):
        return self.map
    
    def get_coords_datatype(self):
        return self.dt 
    
    @dtu.contract(returns='array', xytheta='array')
    def coords_from_xytheta(self, xytheta):
        """ Returns an array with datatype DATATYPE_COORDS """
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['phi'] = dtu.norm_angle(xytheta['theta'])
        res['d'] = xytheta['y'] + self.offset
        return res 
        
    @dtu.contract(returns='array', res='array')
    def xytheta_from_coords(self, res):
        """ Returns an array with datatype dtu.DATATYPE_XYTHETA """
        r = np.zeros((), dtype=dtu.DATATYPE_XYTHETA)
        r['x'] = 0
        r['y'] = res['d'] - self.offset
        r['theta'] = dtu.norm_angle(res['phi'])
        return r

class TemplateStraightLaneWithStop(TemplateStraightLane):
    
    def __init__(self, tile_size, 
                       width_yellow, 
                       width_white,
                       tile_spacing,
                       width_red):
        TemplateStraightLane.__init__(self, tile_size, 
                                      width_yellow, 
                                      width_white,
                                      tile_spacing)
        
        self.map = get_map_straight_lane_with_stop(tile_size, width_yellow, width_white, 
                                                   tile_spacing, width_red)
        self.lane_width = (tile_size - 2*width_white - width_yellow) / 2
        self.dt = TemplateStraightLane.DATATYPE_COORDS
        self.offset = width_yellow/2 + self.lane_width/2
     

@dtu.contract(returns=SegmentsMap)
def get_map_straight_lane(tile_size, width_yellow, width_white, tile_spacing):
    # from inner yellow to inner white
    lane_width = L = (tile_size - 2*width_white - width_yellow) / 2
    msg = """
        width_yellow: %s   
        width_white: %s
        tile "size": %s  (outer white to outer white) 
        lane width: %s  (inner yellow to inner white)
        tile spacing: %s (distance between the center of two nearby tiles)
    """ % (width_yellow, width_white, tile_size, lane_width, tile_spacing)
    dtu.logger.info(msg)
    y1 = + width_yellow/2 + L + width_white
    y2 = + width_yellow/2 + L
    y3 = + width_yellow/2
    y4 = - width_yellow/2
    y5 = - width_yellow/2 - L
    y6 = - width_yellow/2 - L - width_white  
    
    assert_almost_equal(width_white + L + width_yellow + L + width_white, tile_size) 
    
    tile_inter = (tile_spacing - tile_size)/2
    
    # horizon = 2 cells
    num_cells_horizon  = 1
    D = (num_cells_horizon + 0.5)* tile_size

    FRAME = FRAME_TILE
    WHITE = 'white'
    YELLOW = 'yellow'
    S = - tile_size/2
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
    
    # now the tile itself
    def add_quad(cx, cy, L, M, color):
        s = len(points)
        pre = 't%s_' % s
        A = L / 2
        B = M / 2
        points[pre+'t0'] = SegMapPoint(id_frame=FRAME, coords=[-A+cx, -B+cy, 0])
        points[pre+'t1'] = SegMapPoint(id_frame=FRAME, coords=[-A+cx, +B+cy, 0])
        points[pre+'t2'] = SegMapPoint(id_frame=FRAME, coords=[+A+cx, +B+cy, 0])
        points[pre+'t3'] = SegMapPoint(id_frame=FRAME, coords=[+A+cx, -B+cy, 0])
        faces.append(SegMapFace(color=color, 
                                points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    def add_tile(tx, ty):
        L = tile_size + tile_inter*2
        add_quad(tx, ty, L, L, 'black')
        add_quad(tx, ty+tile_size/2, L, L/3, 'black')
        add_quad(tx, ty-tile_size/2, L, L/3, 'black')
#         add_quad(tx, ty, tile_size, 'black')
    
    add_tile(0,0) 
    add_tile(tile_spacing,0) 
    
    gap_len = 0.015
    dash_len = 0.04
    def add_dash(x):
        s = len(points)
        pre = 'dash%s_' % s
        points[pre+'t0'] = SegMapPoint(id_frame=FRAME, coords=[x, y3, 0])
        points[pre+'t1'] = SegMapPoint(id_frame=FRAME, coords=[x, y4, 0])
        points[pre+'t2'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y4, 0])
        points[pre+'t3'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y3, 0])
        faces.append(SegMapFace(color=YELLOW, 
                                points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    
    ngaps = (tile_size * 2) / (gap_len + dash_len) 
    for i in range(int(ngaps)):
        x = i * (gap_len + dash_len) - tile_size/2
        add_dash(x)
    
    from duckietown_msgs.msg import Segment
    segment = Segment() 
    
    segments.append(SegMapSegment(color=segment.WHITE, points=['q1','p1']))
    segments.append(SegMapSegment(color=segment.WHITE, points=['p2','q2']))
    segments.append(SegMapSegment(color=segment.YELLOW, points=['q3','p3']))
    segments.append(SegMapSegment(color=segment.YELLOW, points=['p4','q4']))
    segments.append(SegMapSegment(color=segment.WHITE, points=['q5','p5']))
    segments.append(SegMapSegment(color=segment.WHITE, points=['p6','q6']))
    
    faces.append(SegMapFace(color=WHITE, points=['p1', 'q1', 'q2', 'p2']))
    faces.append(SegMapFace(color=WHITE, points=['p5', 'q5', 'q6', 'p6']))

    data = dict(points=points, segments=segments, faces=faces)

    return SegmentsMap(**data)

@dtu.contract(points='dict', faces='list', id_frame='str', color='str')
def _add_rect(points, faces, x1, y1, x2, y2, id_frame, color):
    assert x2 > x1
    assert y2 > y1
    s = len(points)
    
    pre = '%s_' % s
    points[pre+'t0'] = SegMapPoint(id_frame=id_frame, coords=[x1, y1, 0])
    points[pre+'t1'] = SegMapPoint(id_frame=id_frame, coords=[x1, y2, 0])
    points[pre+'t2'] = SegMapPoint(id_frame=id_frame, coords=[x2, y2, 0])
    points[pre+'t3'] = SegMapPoint(id_frame=id_frame, coords=[x2, y1, 0])
    faces.append(SegMapFace(color=color, 
                            points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    
@dtu.contract(returns=SegmentsMap)
def get_map_straight_lane_with_stop(tile_size, width_yellow, width_white, tile_spacing, width_red):
    straight = get_map_straight_lane(tile_size, width_yellow, width_white, tile_spacing)
    points = straight.points
    faces = straight.faces
    segments = straight.segments
    id_frame = FRAME_TILE
    
    x1 = tile_size/2 - width_red
    y1 = 0
    y2 = tile_size/2
    x2 = tile_size/2 
    color = 'red'
    _add_rect(points, faces, x1, y1, x2, y2, id_frame, color)

    return SegmentsMap(faces=faces, points=points, segments=segments)
    
    

