import duckietown_utils as dtu

from duckietown_utils import m_from_in
from duckietown_segmaps.tiles import get_map_straight_lane


@dtu.unit_test
def f1():
    pass
#
#     sm = get_map_straight_lane( tile_size=m_from_in(12*2),
#                                 width_yellow=m_from_in(1),
#                                 width_white=m_from_in(2))
#     print(sm)

if __name__ == '__main__':
    dtu.run_tests_for_this_module()
