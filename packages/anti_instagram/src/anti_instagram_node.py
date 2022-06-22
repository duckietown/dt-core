#!/usr/bin/env python3

import rospy
import cv2
from multiprocessing import Lock
from image_processing.anti_instagram import AntiInstagram
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AntiInstagramThresholds

from duckietown.dtros import DTROS, NodeType, TopicType


class AntiInstagramNode(DTROS):

    """

    Subscriber:
        ~uncorrected_image/compressed: The uncompressed image coming from the camera


    Publisher:
        ~

    """

    def __init__(self, node_name):

        super(AntiInstagramNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Read parameters

        self._interval = rospy.get_param("~interval")
        self._color_balance_percentage = rospy.get_param("~color_balance_scale")
        self._output_scale = rospy.get_param("~output_scale")
        self._img_size = rospy.get_param("~img_size", None)
        self._top_cutoff = rospy.get_param("~top_cutoff", None)

        # Construct publisher
        self.pub = rospy.Publisher(
            "~thresholds", AntiInstagramThresholds, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        # Construct subscriber
        self.uncorrected_image_subscriber = rospy.Subscriber(
            "~uncorrected_image/compressed",
            CompressedImage,
            self.store_image_msg,
            buff_size=10000000,
            queue_size=1,
        )

        # Initialize Timer
        rospy.Timer(rospy.Duration(self._interval), self.calculate_new_parameters)

        # Initialize objects and data
        self.ai = AntiInstagram()
        self.bridge = CvBridge()
        self.image_msg = None
        self.mutex = Lock()

        # ---
        self.log("Initialized.")

    def store_image_msg(self, image_msg):
        with self.mutex:
            self.image_msg = image_msg

    def decode_image_msg(self):
        with self.mutex:
            try:
                image = self.bridge.compressed_imgmsg_to_cv2(self.image_msg, "bgr8")
            except ValueError as e:
                self.log(f"Anti_instagram cannot decode image: {e}")
        return image

    def calculate_new_parameters(self, event):
        if self.image_msg is None:
            self.log("Waiting for first image!")
            return
        image = self.decode_image_msg()

        if self._img_size is not None:
            h_original, w_original = image.shape[0:2]
            h_requested, w_requested = self._img_size
            if w_requested != w_original or h_requested != h_original:
                image = cv2.resize(image, (w_requested, h_requested), interpolation=cv2.INTER_NEAREST)
        if self._top_cutoff is not None:
            image = image[self._top_cutoff:, :, :]

        self.logdebug(f"anti-instagram sizes: Exp ({self._img_size}, {self._top_cutoff}) Act ({image.shape})")

        (lower_thresholds, higher_thresholds) = self.ai.calculate_color_balance_thresholds(
            image, self._output_scale, self._color_balance_percentage
        )

        # Publish parameters
        msg = AntiInstagramThresholds()
        msg.low = lower_thresholds
        msg.high = higher_thresholds
        self.pub.publish(msg)


if __name__ == "__main__":
    node = AntiInstagramNode(node_name="anti_instagram_node")
    rospy.spin()
