from duckietown_segmaps.maps import SegmentsMap, FRAME_TILE, SegMapPoint,\
    SegMapFace, SegMapSegment
import duckietown_utils as dtu
from duckietown_segmaps.map_localization_template import MapLocalizationTemplate
import numpy as np
from duckietown_utils.coords import DATATYPE_XYTHETA
from numpy.testing.utils import assert_almost_equal

# meters from inches
m_from_in = lambda x: x * 0.0254 

class TemplateStraightLane(MapLocalizationTemplate):
    
    DATATYPE_COORDS = np.dtype([('phi', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_size=m_from_in(12*2), 
                       width_yellow=m_from_in(1), 
                       width_white=m_from_in(2)):
        
        self.map = get_map_straight_lane(tile_size, width_yellow, width_white)
        self.lane_width = (tile_size - 2*width_white - width_yellow) / 2
    
    @dtu.contract(returns=SegmentsMap)
    def get_map(self):
        return self.map
    
    def get_coords_datatype(self):
        return TemplateStraightLane.DATATYPE_COORDS
    
#     @dtu.contract(returns='array[2]', xytheta='array[3]')
    def coords_from_xytheta(self, xytheta):
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['phi'] = xytheta['theta']
        res['d'] = xytheta['y'] + self.lane_width / 2
        return res 
        
    # @dtu.contract(returns='array[3]', res='array[2]')
    def xytheta_from_coords(self, res):
        r = np.zeros((), dtype=DATATYPE_XYTHETA)
        r['x'] = 0
        r['y'] = -self.lane_width / 2 + res['d']
        r['theta'] = res['phi']
        return r  


@dtu.contract(returns=SegmentsMap)
def get_map_straight_lane(tile_size, width_yellow, width_white):
    # from inner yellow to inner white
    L = (tile_size - 2*width_white - width_yellow) / 2
    y1 = + width_yellow/2 + L + width_white
    y2 = + width_yellow/2 + L
    y3 = + width_yellow/2
    y4 = - width_yellow/2
    y5 = - width_yellow/2 - L
    y6 = - width_yellow/2 - L - width_white
    
    assert_almost_equal(width_white + L + width_yellow + L + width_white, tile_size) 
    
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
    def add_tile(cx, cy):
        s = len(points)
        pre = 't%s_' % s
        A = tile_size / 2
        points[pre+'t0'] = SegMapPoint(id_frame=FRAME, coords=[-A+cx, -A+cy, 0])
        points[pre+'t1'] = SegMapPoint(id_frame=FRAME, coords=[-A+cx, +A+cy, 0])
        points[pre+'t2'] = SegMapPoint(id_frame=FRAME, coords=[+A+cx, +A+cy, 0])
        points[pre+'t3'] = SegMapPoint(id_frame=FRAME, coords=[+A+cx, -A+cy, 0])
        faces.append(SegMapFace(color='black', 
                                points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    
    
    add_tile(0,0)
    add_tile(tile_size,0)
    
    gap_len = 0.015
    dash_len = 0.04
    def add_dash(x):
        s = len(points)
        pre = 'dash%s_' % s
        points[pre+'t0'] = SegMapPoint(id_frame=FRAME, coords=[x, y3, 0])
        points[pre+'t1'] = SegMapPoint(id_frame=FRAME, coords=[x, y4, 0])
        points[pre+'t2'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y4, 0])
        points[pre+'t3'] = SegMapPoint(id_frame=FRAME, coords=[x+dash_len, y3, 0])
        faces.append(SegMapFace(color='yellow', 
                                points=[pre+'t0',pre+'t1',pre+'t2',pre+'t3']))
    
    ngaps = (tile_size * 2) / (gap_len + dash_len) 
    for i in range(int(ngaps)):
        x = i * (gap_len + dash_len) - tile_size/2
        add_dash(x)
    
    segments.append(SegMapSegment(color=WHITE, points=['p1','q1']))
    segments.append(SegMapSegment(color=WHITE, points=['p2','q2']))
    segments.append(SegMapSegment(color=YELLOW, points=['p3','q3']))
    segments.append(SegMapSegment(color=YELLOW, points=['p4','q4']))
    segments.append(SegMapSegment(color=WHITE, points=['p5','q5']))
    segments.append(SegMapSegment(color=WHITE, points=['p6','q6']))
    
    
    
    faces.append(SegMapFace(color=WHITE, points=['p1', 'q1', 'q2', 'p2']))
    faces.append(SegMapFace(color=WHITE, points=['p5', 'q5', 'q6', 'p6']))
#     faces.append(SegMapFace(color=YELLOW, points=['p3', 'q3', 'q4', 'p4']))
    


    data = dict(points=points, segments=segments, faces=faces)

    return SegmentsMap(**data)
