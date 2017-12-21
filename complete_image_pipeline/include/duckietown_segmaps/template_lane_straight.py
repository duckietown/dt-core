from duckietown_segmaps.maps import SegmentsMap, FRAME_TILE
import duckietown_utils as dtu


# meters from inches
m_from_in = lambda x: x * 0.0254 

class TemplateStraightLane(object):
    
    def __init__(self, tile_size=m_from_in(12*2), 
                       width_yellow=m_from_in(1), 
                       width_white=m_from_in(2)):
        self.map = get_map_straight_lane(tile_size, width_yellow, width_white)
        
#     def get_coordinates(self):
#         pass

@dtu.contract(returns=SegmentsMap)
def get_map_straight_lane(tile_size, width_yellow, width_white):
    # from inner yellow to inner white
    L = (tile_size - 2*width_white - width_yellow) / 2
    y1 = + L + width_white
    y2 = + L
    y3 = + width_yellow/2
    y4 = - width_yellow/2
    y5 = - L
    y6 = - L - width_white
    
    # horizon = 2 cells
    num_cells_horizon  = 2
    D = num_cells_horizon * tile_size

    FRAME = FRAME_TILE
    WHITE = 'white'
    YELLOW = 'yellow'
    S = - tile_size/2
    points = {
        'p1': [FRAME, [S, y1, 0]],
        'q1': [FRAME, [D, y1, 0]],
        'p2': [FRAME, [S, y2, 0]],
        'q2': [FRAME, [D, y2, 0]],
        'p3': [FRAME, [S, y3, 0]],
        'q3': [FRAME, [D, y3, 0]],
        'p4': [FRAME, [S, y4, 0]],
        'q4': [FRAME, [D, y4, 0]],
        'p5': [FRAME, [S, y5, 0]],
        'q5': [FRAME, [D, y5, 0]],
        'p6': [FRAME, [S, y6, 0]],
        'q6': [FRAME, [D, y6, 0]],
    }
    
    segments = [
        {'points': ['p1', 'q1'],
         'color': WHITE},
        {'points': ['p2', 'q2'],
         'color': WHITE},
        {'points': ['p3', 'q3'],
          'color': YELLOW},
        {'points': ['p4', 'q4'],
         'color': YELLOW},
        {'points': ['p5', 'q5'],
         'color': WHITE},
        {'points': ['p6','q6'],
         'color': WHITE},
    ]
    
    faces = [
        {'color': WHITE, 'points': ['p1', 'q1', 'q2', 'p2']},
        {'color': WHITE, 'points': ['p5', 'q5', 'q6', 'p6']},
        {'color': YELLOW, 'points': ['p3', 'q3', 'q4', 'p4']},
    ]

    data = dict(points=points, segments=segments, faces=faces)

    return SegmentsMap(**data)
