#!/usr/bin/env python3

import os
from typing import List
import unittest

from dt_computer_vision.camera.types import CameraModel
import rospy
import rostest
import yaml
from cv_bridge import CvBridge

# Path to the directory containing images
from dt_computer_vision_tests.optical_flow_tests.scenario_1 import (
    generate_translated_images_sequence,
    yaml_homography_fpath,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Range

CM_TO_METERS = 1e-2

class TestOpticalFlowNode(unittest.TestCase):
    """
    Integration test for the optical flow node.
    This test takes as input a single image that includes the distance to the object in the image file name.

    It then generates a list of images that move the image rightwards, simulating a motion of the drone.
    The optical flow node is then fed these images and the range to the object.
    The optical flow node should then output a visual odometry message that contains the correct x and y velocities.

    If executed as a standalone script this test will use the namespace "test".
    If executed as a rostest this test will use the namespace provided by the launch file.

    NOTE: This test uses the scenario1 files copied from the lib_dt_computer_vision_tests package.

    TODO: Refactor this test to use the scenario1 files from the dt_computer_vision_tests package.
    """

    test_script_dir = os.path.dirname(os.path.abspath(__file__))

    distance_cm = 50  # 50 cm
    image_frequency = 30

    base_image_fpath = test_script_dir + f"/scenario1/{distance_cm}cm.png"
    yaml_camera_calibration_fpath = (
        test_script_dir + "/scenario1/calibration-intrinsic-dd24.yaml"
    )
    yaml_extrinsics_fpath = test_script_dir + "/scenario1/homography.yaml"

    robot_extrinsics_dir = "/data/config/calibrations/camera_extrinsic"
    os.makedirs(robot_extrinsics_dir, exist_ok=True)

    robot_extrinsics_path = robot_extrinsics_dir + "/default.yaml"

    def setUp(self):
        rospy.init_node("test_optical_flow_node", anonymous=True, log_level=rospy.DEBUG)
        self.bridge = CvBridge()

        self.config = {
            "process_frequency": 20,
            "track_len": 10,
            "detect_interval": 5,
            "img_scale": 0.25,
        }
        self.create_homography_file(
            self.robot_extrinsics_path,
            original_homography_fpath=self.yaml_extrinsics_fpath,
        )

        # Set ros parameters of optical_flow node using the config

        # TODO: Get the namespace from the launch file
        self._namespace = rospy.get_namespace().strip("/")

        if self._namespace == "":
            self._namespace = "test"

        self.image_pub = rospy.Publisher(
            f"/{self._namespace}/camera_node/image/compressed",
            CompressedImage,
            queue_size=1,
        )
        self.range_pub = rospy.Publisher(
            f"/{self._namespace}/sensor/range", Range, queue_size=1
        )
        self.camera_info_pub = rospy.Publisher(
            f"/{self._namespace}/camera_node/camera_info", CameraInfo, queue_size=1
        )

        self.odom_received = False
        self.odom_sub = rospy.Subscriber(
            f"/{self._namespace}/optical_flow_node/visual_odometry",
            Odometry,
            self.odom_callback,
        )
        self.odometry_messages: List[Odometry] = []

        rospy.loginfo("Subscribing to the following topics:")
        rospy.loginfo(f"- Image topic: /{self._namespace}/camera_node/image/compressed")
        rospy.loginfo(f"- Range topic: /{self._namespace}/sensor/range")
        rospy.loginfo(
            f"- Camera info topic: /{self._namespace}/camera_node/camera_info"
        )

        rospy.loginfo("Publishing to the following topics:")
        rospy.loginfo(
            f"- Odometry topic: /{self._namespace}/optical_flow_node/visual_odometry"
        )

    def odom_callback(self, msg: Odometry):
        self.odom_received = True
        self.odometry_messages.append(msg)
        rospy.loginfo(f"Odometry message received: {msg}")

    def create_homography_file(
        self, robot_homography_fpath, original_homography_fpath=yaml_homography_fpath
    ):
        # Load the homography from the file
        with open(original_homography_fpath, "r") as file:
            yaml_content = file.read()
            homography_file = yaml.safe_load(yaml_content)

        rospy.loginfo(
            f"Loaded the following homography from file {original_homography_fpath}: {homography_file}"
        )

        # Write the homography to the path expected by the optical flow node
        with open(robot_homography_fpath, "w") as file:
            file.write(yaml_content)

    def create_camera_info_message(self, file_path):
        camera_info = CameraInfo()
        rospy.loginfo(f"Creating camera info message from file {file_path}...")

        with open(file_path, "r") as file:
            yaml_content = file.read()
            camera = CameraModel.from_ros_calibration(yaml_content)

            camera_info.height = camera.height
            camera_info.width = camera.width
            camera_info.K = camera.K.flatten().tolist()
            camera_info.D = camera.D.flatten()
            camera_info.R = camera.R.flatten()
            camera_info.P = camera.P.flatten()

        rospy.loginfo(
            f"Camera info message created from file {file_path}: {camera_info}"
        )

        return camera_info

    def test_optical_flow_node(self):
        rospy.loginfo(
            f"Using image file {self.base_image_fpath} for testing optical flow node in a {self.distance_cm}cm range scenario."
        )

        # Generate a sequence of translated images
        images = generate_translated_images_sequence(self.base_image_fpath)

        # Load camera model from YAML file
        camera_info = self.create_camera_info_message(
            self.yaml_camera_calibration_fpath
        )

        # Create and publish range message
        range_msg = Range()
        range_msg.range = self.distance_cm * CM_TO_METERS

        # Publish the images read from the directory
        for img in images:
            if img is None:
                continue
            compressed_image = self.bridge.cv2_to_compressed_imgmsg(img)
            # Sync the camera info and image messages
            camera_info.header.stamp = rospy.Time.now()
            compressed_image.header.stamp = rospy.Time.now()

            self.range_pub.publish(range_msg)

            self.camera_info_pub.publish(camera_info)
            self.image_pub.publish(compressed_image)
            rospy.sleep(1 / self.image_frequency)

        # Check if odometry message is received
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not self.odom_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        self.assertTrue(self.odom_received, "Odometry message not received")

        # Verify that all odometry messages have a float value for the x and y linear velocities
        for msg in self.odometry_messages:
            self.assertIsInstance(
                msg.twist.twist.linear.x,
                float,
                f"Expected float, got {type(msg.twist.twist.linear.x)}",
            )
            self.assertIsInstance(
                msg.twist.twist.linear.y,
                float,
                f"Expected float, got {type(msg.twist.twist.linear.y)}",
            )


if __name__ == "__main__":
    rostest.rosrun("optical_flow", "test_optical_flow_node", TestOpticalFlowNode)
