from collections import OrderedDict
import os

from .image_simulation import simulate_image
from duckietown_segmaps import FAMILY_SEGMAPS
import duckietown_utils as dtu
from duckietown_utils.cli import D8App
from easy_algo import get_easy_algo_db
from ground_projection import GroundProjection,  NoHomographyInfoAvailable
import numpy as np
from pi_camera import NoCameraInfoAvailable


__all__ = [
    'ValidateCalibration',
]

class ValidateCalibration(D8App):
    """

        This program validates the intrinsic/extrinsics calibrations.

    """

    usage = """

Use as follows:

    $ rosrun complete_image_pipeline validate_calibration [robot names]

Example:

    $ rosrun complete_image_pipeline validate_calibration shamrock emma

"""

    def define_program_options(self, params):
        params.add_string('output', short='o', help='Output dir',
                          default='out-validate_calibration')
        params.accept_extra()

    def go(self):
        extra = self.options.get_extra()
        db = get_easy_algo_db()

        if len(extra) == 0:
            query = '*'
        else:
            query = extra

        robots = db.query('robot', query)
        self.debug('robots: %s' % sorted(robots))


        actual_map_name =  'DT17_scenario_four_way'

        out = self.options.output
        create_visuals(robots, actual_map_name, out)


def create_visuals(robots, actual_map_name, out):
    db = get_easy_algo_db()
    actual_map = db.create_instance(FAMILY_SEGMAPS, actual_map_name)
    res = OrderedDict()
    res2 = OrderedDict()

    for i, robot_name in enumerate(sorted(robots)):
        dtu.logger.info('%d/%d: %s' % (i, len(robots), robot_name))
        try:
            gp = GroundProjection(robot_name)
        except (NoCameraInfoAvailable, NoHomographyInfoAvailable) as e:
            dtu.logger.warning('skipping %r: %s' % (robot_name, e))
        gpg = gp.get_ground_projection_geometry()
        pose = np.eye(3)
        rectified_synthetic, distorted = \
            simulate_image(actual_map, pose, gpg, blur_sigma=1)
        res[robot_name] = rectified_synthetic
        res2[robot_name] = distorted
    output = os.path.join(out, 'distorted')
    dtu.write_bgr_images_as_jpgs(res2, output)
    output = os.path.join(out, 'rectified')
    dtu.write_bgr_images_as_jpgs(res, output)
