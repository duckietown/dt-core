from comptests import comptest, run_module_tests

from duckietown_segmaps.template_lane_straight import get_map_straight_lane
from duckietown_utils import m_from_in


@comptest
def f1():
   
    sm = get_map_straight_lane( tile_size=m_from_in(12*2), 
                                width_yellow=m_from_in(1), 
                                width_white=m_from_in(2))
    print(sm)

if __name__ == '__main__':
    run_module_tests()