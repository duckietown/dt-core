import argparse
import socket
from os.path import basename, expanduser, isfile, join, splitext

import cv2
import numpy as np
import rospy
from duckietown_utils import (d8_compressed_image_from_cv_image, get_base_name,
                              get_duckiefleet_root, load_camera_intrinsics,
                              load_homography, load_map, logger, rectify,
                              rgb_from_ros, yaml_load)
from sensor_msgs.msg import CompressedImage


class CameraImageRectifier():

    def __init__(self, robot_name=''):
        # Load camera calibration parameters
        self.intrinsics = load_camera_intrinsics(robot_name)

    def process_image(self, image):
        return d8_compressed_image_from_cv_image(rectify(image[..., ::-1], self.intrinsics)).data
