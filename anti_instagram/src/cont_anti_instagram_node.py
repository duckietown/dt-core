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
import os

"""
This node subscribed to the uncorrected images from the camera. Within a certain time interval (defined from
    commandline) this node calculates and publishes the trafo based on the old anti instagram method.
"""


class ContAntiInstagramNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # TODO verify if required?
        # self.active = True
        # self.locked = False

        # Initialize publishers and subscribers
        self.pub_trafo = rospy.Publisher(
            "~transform", AntiInstagramTransform, queue_size=1)
        self.pub_health = rospy.Publisher(
            "~health", AntiInstagramHealth, queue_size=1, latch=True)

        self.sub_image = rospy.Subscriber(
            #"/duckierick/camera_node/image/compressed", CompressedImage, self.cbNewImage, queue_size=1)
            "/tesla/camera_node/image/compressed", CompressedImage, self.cbNewImage, queue_size=1)
            #"~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)


        # TODO verify name of parameter
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', True)

        # Read parameters
        self.interval = self.setupParameter("~ai_interval", 10)

        # Initialize health message
        self.health = AntiInstagramHealth()

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it

        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        self.image_msg = None

        # timer for continuous image process
        self.timer = rospy.Timer(rospy.Duration(self.interval), self.processImage)

        # TODO write to file within git folder
        self.file = open('/home/milan/output_cont_ai_node.txt', 'a+')
        self.file.write('\nCONT_ANTI_INSTAGRAM_NODE:\n')

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)#Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbNewImage(self, image_msg):
        # memorize image
        self.image_msg = image_msg


    def processImage(self, event):
        print('processImg called!')

        # if we have seen an image:
        if self.image_msg is not None:
            rospy.loginfo('ai: Computing color transform...')
            tk = TimeKeeper(self.image_msg)

            try:
                cv_image = image_cv_from_jpg(self.image_msg.data)
            except ValueError as e:
                rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
                return

            tk.completed('converted')

            self.ai.calculateTransform(cv_image)

            tk.completed('calculateTransform')

            # if health is much below the threshold value, do not update the color correction and log it.
            if self.ai.health <= 0.001:
                # health is not good

                rospy.loginfo("Health is not good")

            else:
                self.health.J1 = self.ai.health
                self.transform.s[0], self.transform.s[1], self.transform.s[2] = self.ai.shift
                self.transform.s[3], self.transform.s[4], self.transform.s[5] = self.ai.scale

                self.pub_health.publish(self.health)
                self.pub_trafo.publish(self.transform)
                rospy.loginfo('ai: Color transform published.')

                # write latest trafo to file
                self.file.write('shift:\n' + str(self.ai.shift) + '\nscale:\n'  + str(self.ai.scale) + '\nhealth:\n' + str(self.ai.health) + '\n \n')




if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('cont_anti_instagram_node', anonymous=False)

    # Create the NodeName object
    node = ContAntiInstagramNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
