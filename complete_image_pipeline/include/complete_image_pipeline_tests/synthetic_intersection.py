from comptests.registrar import comptest, run_module_tests

import numpy as np

from .synthetic import test_synthetic_phi

template = 'DT17_ETH_straight_stop'
robot_name = 'flitzer'
line_detector_name = 'baseline'
# image_prep_name = 'prep_200_100'
image_prep_name = 'baseline'
# lane_filter_names = ['baseline', 'generic_straight']
lane_filter_names = []
lane_filter_names += ['generic_straight_stop']
# lane_filter_names += ['baseline']
raise_if_error_too_large = True

max_phi_err = np.deg2rad(5)
max_d_err = 0.021



dirn = lambda _: 'out-synthetic/%s' % _

@comptest
def stopline_zero_zerophi():
    d = 0
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('stopline_zero_zerophi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi , outd)

if __name__ == '__main__':
    run_module_tests()