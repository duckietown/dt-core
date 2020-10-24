from dataclasses import dataclass

from easy_regression.processors.localization_pipeline import read_camera_info_from_bag
from image_processing.calibration_utils import get_camera_info_for_robot, get_homography_for_robot
from image_processing.ground_projection_geometry import GroundProjectionGeometry
from image_processing.rectification import Rectify
from sensor_msgs.msg import CameraInfo
import duckietown_utils as dtu


@dataclass
class RobotCameraGeometry:
    rectifier: Rectify
    gpg: GroundProjectionGeometry


def get_robot_camera_geometry(robot_name: str) -> RobotCameraGeometry:
    ci: CameraInfo = get_camera_info_for_robot(robot_name)
    K = get_homography_for_robot(robot_name)

    rectifier = Rectify(ci)
    gpg = GroundProjectionGeometry(ci.width, ci.height, K)
    return RobotCameraGeometry(rectifier, gpg)


def get_robot_camera_geometry_from_log(brp: dtu.BagReadProxy) -> RobotCameraGeometry:
    robot_name = dtu.which_robot(brp)

    K = get_homography_for_robot(robot_name)
    ci = read_camera_info_from_bag(brp)
    rectifier = Rectify(ci)
    gpg = GroundProjectionGeometry(ci.width, ci.height, K)
    return RobotCameraGeometry(rectifier, gpg)
