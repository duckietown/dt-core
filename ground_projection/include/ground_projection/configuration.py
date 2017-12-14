import duckietown_utils as dtu
import os
import yaml

class NoHomographyInfoAvailable(dtu.DTException):
    pass

class InvalidHomographyInfo(dtu.DTException):
    pass


def get_homography_default():
    """ Returns a nominal homography """
    return get_homography_for_robot('default')

@dtu.contract(robot_name=str, returns='array[3x3]')
def get_homography_for_robot(robot_name):
    # find the file
    fn = get_homography_info_config_file(robot_name)
    
    # load the YAML
    data = dtu.yaml_load_file(fn)
    
    # convert the YAML
    homography = homography_from_yaml(data) 
    
    check_homography_sane_for_DB17(homography)
    
    return homography

import numpy as np
    
@dtu.contract(data=dict)
def homography_from_yaml(data):
    try:
        h = data['homography']
        res = np.array(h).reshape((3,3))
        return res
    except Exception as e:
        msg = 'Could not interpret data:'
        msg += '\n\n' + dtu.indent(yaml.dump(data), '   ')
        dtu.raise_wrapped(InvalidHomographyInfo, e, msg)

def get_homography_info_config_file(robot_name):
    df = dtu.get_duckiefleet_root()
    
    # Load camera information
    fn = os.path.join(df, 'calibrations', 'camera_extrinsic', robot_name + '.yaml')
    
    if not os.path.exists(fn):
        msg = 'Cannot find homography file for robot %r;\n%s' % (robot_name, fn) 
        raise NoHomographyInfoAvailable(msg)
    
    return fn

def check_homography_sane_for_DB17(homography):
    # TODO: to write
    pass

            