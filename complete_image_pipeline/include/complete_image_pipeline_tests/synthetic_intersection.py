import duckietown_utils as dtu
import numpy as np

from .synthetic import test_synthetic_phi


# template = 'DT17_straight_stop'
actual_map_name =  'DT17_four_way'
template = 'DT17_four_way'
robot_name = 'flitzer'
line_detector_name = 'baseline'
# image_prep_name = 'prep_200_100'
image_prep_name = 'baseline'
# lane_filter_names = ['baseline', 'generic_straight']
lane_filter_names = []
# lane_filter_names += ['generic_fourway']
lane_filter_names += ['moregeneric_fourway']
# lane_filter_names += ['baseline']
raise_if_error_too_large = True

max_phi_err = np.deg2rad(5)
max_d_err = 0.021


dirn = lambda _: 'out-synthetic/%s' % _

@dtu.unit_test
def intersection_zero_zerophi():
    d = 0
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('intersection_zero_zerophi-' + lane_filter_name)
        test_synthetic_phi(actual_map_name, template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi , outd)

@dtu.unit_test
def intersection_zero_negphi():
    d = 0
    phi = np.deg2rad(-13)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('intersection_zero_negphi-' + lane_filter_name)
        test_synthetic_phi(actual_map_name, template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi , outd)

@dtu.unit_test
def intersection_zero_posphi():
    d = 0
    phi = np.deg2rad(+13)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('intersection_zero_posphi-' + lane_filter_name)
        test_synthetic_phi(actual_map_name, template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi , outd)

@dtu.unit_test
def intersection_zero_posphi2():
    d = 0
    phi = np.deg2rad(+10)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('intersection_zero_posphi2-' + lane_filter_name)
        test_synthetic_phi(actual_map_name, template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi , outd)

if __name__ == '__main__':
    dtu.run_tests_for_this_module()
    