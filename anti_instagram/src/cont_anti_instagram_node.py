#!/usr/bin/env python
import rospy
from anti_instagram.AntiInstagram import *
from cv_bridge import CvBridge  # @UnresolvedImport
# @UnresolvedImport
from duckietown_msgs.msg import (AntiInstagramHealth, AntiInstagramTransform,
                                 BoolStamped)
from duckietown_utils.jpg import image_cv_from_jpg
from line_detector.timekeeper import TimeKeeper
from sensor_msgs.msg import CompressedImage, Image  # @UnresolvedImport
import numpy as np
import rospy



class ContAntiInstagramNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # TODO verify if required?
        # self.active = True
        # self.locked = False

        # Initialize publishers and subscribers
        self.pub_trafo = rospy.Publisher(
            "~transform", AntiInstagramTransform, queue_size=1)

        self.sub_image = rospy.Subscriber(
            "/duckierick/camera_node/image/compressed", CompressedImage, self.cbNewImage, queue_size=1)
            #"~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)


        # TODO verify name of parameter
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', True)

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it

        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        self.image_msg = None

        # timer for continuous image process
        self.timer = rospy.Timer(rospy.Duration(10), processImage)


    def cbNewImage(self, image_msg):
        # memorize image
        self.image_msg = image_msg


    def processImage(self):
        print('processImg called!')


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('cont_anti_instagram_node', anonymous=False)

    # Create the NodeName object
    node = ContAntiInstagramNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
