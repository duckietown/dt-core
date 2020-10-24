import os
import shutil
import time
from dataclasses import dataclass

import numpy as np
import yaml

import duckietown_utils as dtu
from sensor_msgs.msg import CameraInfo


class NoHomographyInfoAvailable(dtu.DTException):
    pass


class InvalidHomographyInfo(dtu.DTException):
    pass


def get_homography_default():
    """ Returns a nominal homography """
    return get_homography_for_robot("default")


def get_homography_for_robot(robot_name: str) -> np.ndarray:
    dtu.check_isinstance(robot_name, str)
    # find the file

    fn = get_homography_info_config_file(robot_name)

    # load the YAML
    data = dtu.yaml_load_file(fn, plain_yaml=True)  # True means "plain"

    # convert the YAML
    homography = homography_from_yaml(data)

    return homography


def get_homography_info_config_file(robot_name: str) -> str:
    """
    Raises NoHomographyInfoAvailable.

    :param robot_name:
    :return:
    """
    strict = False
    roots = [
        os.path.join(dtu.get_duckiefleet_root(), "calibrations"),
        os.path.join(dtu.get_ros_package_path("duckietown"), "config", "baseline", "calibration"),
    ]

    found = []
    for df in roots:
        # Load camera information
        fn = os.path.join(df, "camera_extrinsic", robot_name + ".yaml")
        fn_default = os.path.join(df, "camera_extrinsic", "default.yaml")
        if os.path.exists(fn):
            found.append(fn)
            msg = "Using filename %s" % fn
            print(msg)
            dtu.logger.info(msg)
        elif os.path.exists(fn_default):
            found.append(fn_default)
            msg = "Using filename %s" % fn_default
            print(msg)
            dtu.logger.info(msg)

    if len(found) == 0:
        msg = "Cannot find homography file for robot %r;\n%s" % (robot_name, roots)
        print(msg)
        raise NoHomographyInfoAvailable(msg)
    elif len(found) == 1:
        return found[0]
    else:
        msg = "Found more than one configuration file: \n%s" % "\n".join(found)
        msg += "\n Please delete one of those."
        print(msg)
        if strict:
            raise Exception(msg)
        else:
            dtu.logger.error(msg)
            return found[0]


def homography_from_yaml(data: dict) -> np.array:
    try:
        h = data["homography"]
        res = np.array(h).reshape((3, 3))
        return res
    except Exception as e:
        msg = "Could not interpret data:"
        msg += "\n\n" + dtu.indent(yaml.dump(data), "   ")
        dtu.logger.error(msg)
        dtu.raise_wrapped(InvalidHomographyInfo, e, msg)
        raise


def save_homography(H: np.array, robot_name: str) -> None:
    dtu.logger.info("Homography:\n %s" % H)

    # Check if specific point in matrix is larger than zero (this would definitly mean we're having a
    # corrupted rotation matrix)
    if H[1][2] > 0:
        msg = "WARNING: Homography could be corrupt."
        msg += "\n %s" % H
        raise Exception(msg)

    ob = {"homography": sum(H.reshape(9, 1).tolist(), [])}

    s = yaml.dump(ob)
    s += "\n# Calibrated on "
    localTime = "" + dtu.format_time_as_YYYY_MM_DD(time.time())
    s += localTime

    fn = get_extrinsics_filename(robot_name)

    dtu.write_data_to_file(s, fn)


def get_extrinsics_filename(robot_name: str) -> str:
    fn = dtu.get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + robot_name + ".yaml"
    return fn


def disable_old_homography(robot_name: str):
    fn = get_extrinsics_filename(robot_name)
    if os.path.exists(fn):
        fn2 = None
        for i in range(100):
            fn2 = fn + ".disabled.%03d" % i
            if not os.path.exists(fn2):
                break
        msg = (
            "Disabling old homography - so that if this fails it is clear it failed.\n Backup saved as "
            "%s" % fn2
        )
        dtu.logger.warning(msg)
        shutil.move(fn, fn2)


class NoCameraInfoAvailable(dtu.DTException):
    pass


class InvalidCameraInfo(dtu.DTException):
    pass


def get_camera_info_default() -> CameraInfo:
    """ Returns a nominal CameraInfo """
    return get_camera_info_for_robot("default")


default_camera_info = """
image_width: 640
image_height: 480
camera_name: /shamrock/rosberrypi_cam
camera_matrix:
  rows: 3
  cols: 3
  data: [305.5718893575089, 0, 303.0797142544728, 0, 308.8338858195428, 231.8845403702499, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.2944667743901807, 0.0701431287084318, 0.0005859930422629722, -0.0006697840226199427, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [220.2460277141687, 0, 301.8668918355899, 0, 0, 238.6758484095299, 227.0880056118307, 0, 0, 0, 1, 0]
"""


def get_camera_info_for_robot(robot_name: str) -> CameraInfo:
    """
        Returns a CameraInfo object for the given robot.
        This is in a good format to pass to PinholeCameraModel:
            self.pcm = PinholeCameraModel()
            self.pcm.fromCameraInfo(self.ci)
        The fields are simply lists (not array or matrix).
        Raises:
            NoCameraInfoAvailable  if no info available
            InvalidCameraInfo   if the info exists but is invalid
    """

    if robot_name == dtu.DuckietownConstants.ROBOT_NAME_FOR_TESTS:
        calib_data = dtu.yaml_load(default_camera_info)
        fn = None
    else:
        # find the file
        fn = get_camera_info_config_file(robot_name)

        # load the YAML

        calib_data = dtu.yaml_load_file(fn, plain_yaml=True)

    # convert the YAML
    try:
        camera_info = camera_info_from_yaml(calib_data)
    except InvalidCameraInfo as e:
        msg = f"Invalid data in file {fn}"
        raise InvalidCameraInfo(msg) from e

    check_camera_info_sane_for_DB17(camera_info)

    return camera_info


def check_camera_info_sane_for_DB17(camera_info: CameraInfo):
    """ Raises an exception if the calibration is way off with respect
        to platform DVB17 """

    # TODO: to write
    pass


def camera_info_from_yaml(calib_data: dict) -> CameraInfo:
    try:
        cam_info = CameraInfo()
        cam_info.width = calib_data["image_width"]
        cam_info.height = calib_data["image_height"]
        #         cam_info.K = np.matrix(calib_data['camera_matrix']['data']).reshape((3,3))
        #         cam_info.D = np.matrix(calib_data['distortion_coefficients']['data']).reshape((1,5))
        #         cam_info.R = np.matrix(calib_data['rectification_matrix']['data']).reshape((3,3))
        #         cam_info.P = np.matrix(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.K = calib_data["camera_matrix"]["data"]
        cam_info.D = calib_data["distortion_coefficients"]["data"]
        cam_info.R = calib_data["rectification_matrix"]["data"]
        cam_info.P = calib_data["projection_matrix"]["data"]

        cam_info.distortion_model = calib_data["distortion_model"]
        return cam_info
    except Exception as e:
        msg = "Could not interpret data:"
        msg += "\n\n" + dtu.indent(yaml.dump(calib_data), "   ")
        dtu.raise_wrapped(InvalidCameraInfo, e, msg)


def get_camera_info_config_file(robot_name: str) -> str:
    roots = [
        os.path.join(dtu.get_duckiefleet_root(), "calibrations"),
        os.path.join(dtu.get_ros_package_path("duckietown"), "config", "baseline", "calibration"),
    ]

    for df in roots:
        # Load camera information
        fn = os.path.join(df, "camera_intrinsic", robot_name + ".yaml")
        fn_default = os.path.join(df, "camera_intrinsic", "default.yaml")
        if os.path.exists(fn):
            return fn
        elif os.path.exists(fn_default):
            return fn_default
        else:
            dtu.logger.debug("%s does not exist and neither does %s" % (fn, fn_default))

    msg = "Cannot find intrinsic file for robot %r;\n%s" % (robot_name, roots)
    raise NoCameraInfoAvailable(msg)


def load_camera_info_2(filename: str) -> CameraInfo:
    with open(filename, "r") as f:
        calib_data = yaml.load(f)
    cam_info = CameraInfo()
    cam_info.width = calib_data["image_width"]
    cam_info.height = calib_data["image_height"]
    cam_info.K = calib_data["camera_matrix"]["data"]
    cam_info.D = calib_data["distortion_coefficients"]["data"]
    cam_info.R = calib_data["rectification_matrix"]["data"]
    cam_info.P = calib_data["projection_matrix"]["data"]
    cam_info.distortion_model = calib_data["distortion_model"]
    return cam_info
