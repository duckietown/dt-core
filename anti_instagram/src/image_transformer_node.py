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

"""
This node subscribed to the uncorrected images from the camera and corrects these images
    with the latest stored transform.
It also subscribed to the trafo message coming from the Anti-Instagram node. When received,
    it updates the stored transform parameters.
"""

class ImageTransformerNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # TODO verify if required?
        #self.active = True
        #self.locked = False

        # Initialize publishers and subscribers
        self.pub_image = rospy.Publisher(
            "~corrected_image", Image, queue_size=1)

        self.sub_image = rospy.Subscriber(
            "~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)
        self.sub_trafo = rospy.Subscriber(
            "~transform", AntiInstagramTransform, self.cbNewTrafo, queue_size=1)

        # TODO verify name of parameter
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', True)

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it

        self.corrected_image = Image()
        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        self.image_msg = None



    def cbNewImage(self, image_msg):
        # memorize image
        self.image_msg = image_msg

        tk = TimeKeeper(image_msg)
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        corrected_image_cv2 = self.ai.applyTransform(cv_image)
        tk.completed('applyTransform')

        # TODO integrate clipping into ai?
        corrected_image_cv2 = np.clip(
            corrected_image_cv2, 0, 255).astype(np.uint8)
        self.corrected_image = self.bridge.cv2_to_imgmsg(
            corrected_image_cv2, "bgr8")
        tk.completed('encode')

        self.pub_image.publish(self.corrected_image)
        tk.completed('published')

        if self.verbose:
            rospy.loginfo('ai:\n' + tk.getall())

    def cbNewTrafo(self, trafo_msg):
        # memorize transform message
        self.transform = trafo_msg

        # store transform to the Anti-Instagram instance
        self.ai.shift = trafo_msg.s[0:3]         #copied from line_detector2 ldn.py
        self.ai.scale = trafo_msg.s[3:6]

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('image_transformer_node', anonymous=False)

    # Create the NodeName object
    node = ImageTransformerNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
