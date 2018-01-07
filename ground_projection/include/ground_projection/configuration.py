import os

import yaml

import duckietown_utils as dtu
import numpy as np


class NoHomographyInfoAvailable(dtu.DTException):
    pass


class InvalidHomographyInfo(dtu.DTException):
    pass


def get_homography_default():
    """ Returns a nominal homography """
    return get_homography_for_robot('default')


# shamrock
homography_default = """
homography: [-5.828719e-05, -0.0001358896, -0.2350442, 0.001113641, -2.290353e-05, -0.3695509, -0.0003339684, -0.007747321, 1]
"""


@dtu.contract(robot_name=str, returns='array[3x3]')
def get_homography_for_robot(robot_name):
    # find the file
    if robot_name == dtu.DuckietownConstants.ROBOT_NAME_FOR_TESTS:
        data = dtu.yaml_load(homography_default)
    else:
        fn = get_homography_info_config_file(robot_name)

        # load the YAML
        data = dtu.yaml_load_file(fn)

    # convert the YAML
    homography = homography_from_yaml(data)

    check_homography_sane_for_DB17(homography)

    return homography


@dtu.contract(data=dict)
def homography_from_yaml(data):
    try:
        h = data['homography']
        res = np.array(h).reshape((3, 3))
        return res
    except Exception as e:
        msg = 'Could not interpret data:'
        msg += '\n\n' + dtu.indent(yaml.dump(data), '   ')
        dtu.raise_wrapped(InvalidHomographyInfo, e, msg)


def get_homography_info_config_file(robot_name):
    roots = [os.path.join(dtu.get_duckiefleet_root(), 'calibrations'),
             os.path.join(dtu.get_ros_package_path('duckietown'), 'config', 'baseline', 'calibration')]

    for df in roots:
    # Load camera information
        fn = os.path.join(df, 'camera_extrinsic', robot_name + '.yaml')
        if os.path.exists(fn):
            return fn

    msg = 'Cannot find homography file for robot %r;\n%s' % (robot_name, roots)
    raise NoHomographyInfoAvailable(msg)


def check_homography_sane_for_DB17(homography):
    # TODO: to write
    pass

