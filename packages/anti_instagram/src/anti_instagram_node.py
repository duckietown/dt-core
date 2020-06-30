#!/usr/bin/env python

import rospy

from image_processing import AntiInstagram
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

        super(AntiInstagramNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        # Read parameters

        self._interval = rospy.get_param("~interval")
        self._color_balance_percentage = rospy.get_param("~color_balance_scale")
        self._output_scale = rospy.get_param("~output_scale")
        self._calculation_scale = rospy.get_param("~resize_factor")

        # Construct publisher
        self.pub = rospy.Publisher(
            "~thresholds",
            AntiInstagramThresholds,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        # Construct subscriber
        self.uncorrected_image_subscriber = rospy.Subscriber(
            '~uncorrected_image/compressed',
            CompressedImage,
            self.process_image
        )

        # Initialize Timer
        rospy.Timer(rospy.Duration(self.interval),
                    self.calculate_new_parameters)

        # Initialize objects and data
        self.ai = AntiInstagram()
        self.bridge = CvBridge()
        self.image = None

        # ---
        self.log("Initialized.")



    def process_image(self, image_msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            self.image = image
        except ValueError as e:
            self.log('Anti_instagram cannot decode image: %s' % e)
            return

    def calculate_new_parameters(self, event):
        if self.image is None:
            self.log("[%s] Waiting for first image!")
            return

        (lower_thresholds, higher_thresholds) =
            self.ai.calculate_color_balance_thresholds(image,
                                                self.calculation_scale,
                                                self.color_balance_percentage)

        # Publish parameters
        msg = AntiInstagramThresholds()
        msg.low = lower_thresholds
        msg.high = higher_thresholds
        self.pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('anti_instagram_node', anonymous=False)
    node = AntiInstagramNode()
    rospy.spin()
