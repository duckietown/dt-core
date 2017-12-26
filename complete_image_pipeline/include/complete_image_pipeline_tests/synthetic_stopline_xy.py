import duckietown_utils as dtu
import numpy as np
from .synthetic import test_synthetic
from geometry import SE2_from_translation_angle

actual_map_name =  'DT17_four_way'
template = 'DT17_template_xy_stopline'
robot_name = 'flitzer'
line_detector_name = 'baseline'
# image_prep_name = 'prep_200_100'
image_prep_name = 'baseline'
# lane_filter_names = ['baseline', 'generic_straight']
lane_filter_names = []
lane_filter_names += ['moregeneric_xy_stopline']
# lane_filter_names += ['baseline']
raise_if_error_too_large = True

max_phi_err = np.deg2rad(5)
max_d_err = 0.021


dirn = lambda _: 'out-synthetic/%s' % _

@dtu.unit_test
def stopline_zero_zero():
    pose_or_location = SE2_from_translation_angle([0, -0.10], np.deg2rad(5))
 
    for lane_filter_name in lane_filter_names:    
        outd = dirn('stopline_zero_zerophi-' + lane_filter_name)
        test_synthetic(actual_map_name, template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, pose_or_location, outd)


if __name__ == '__main__':
    dtu.run_tests_for_this_module()
    