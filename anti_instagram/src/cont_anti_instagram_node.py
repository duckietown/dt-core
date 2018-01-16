#!/usr/bin/env python
import rospy
from anti_instagram.AntiInstagram_rebuild import *
from anti_instagram.kmeans_rebuild import *
from cv_bridge import CvBridge  # @UnresolvedImport
# @UnresolvedImport
from duckietown_msgs.msg import (AntiInstagramHealth, AntiInstagramTransform, AntiInstagramTransform_CB, BoolStamped)
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
        robot_name = rospy.get_param("~veh", "") #to read the name always reliably

        # TODO verify if required?
        # self.active = True
        # self.locked = False

        # Initialize publishers and subscribers
        self.pub_trafo = rospy.Publisher(
            #"~transform", AntiInstagramTransform, queue_size=1)
            '/{}/anti_instagram_node/transform'.format(robot_name), AntiInstagramTransform, queue_size=1)
        self.pub_trafo_CB = rospy.Publisher(
            "~colorBalanceTrafo", AntiInstagramTransform_CB, queue_size=1)
        self.pub_health = rospy.Publisher(
            "~health", AntiInstagramHealth, queue_size=1, latch=True)

        self.sub_image = rospy.Subscriber(
            # "/duckierick/camera_node/image/compressed", CompressedImage, self.cbNewImage, queue_size=1)
            '/{}/camera_node/image/compressed'.format(robot_name), CompressedImage, self.cbNewImage, queue_size=1)
            #"~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)


        # TODO verify name of parameter
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', True)

        # Read parameters
        self.interval = self.setupParameter("~ai_interval", 10)
        self.fancyGeom = self.setupParameter("~fancyGeom", False)
        self.n_centers = self.setupParameter("~n_centers", 10)
        self.blur = self.setupParameter("~blur", 'median')
        self.resize = self.setupParameter("~resize", 0.2)
        self.blur_kernel = self.setupParameter("~blur_kernel", 5)
        self.cb_percentage = self.setupParameter("~cb_percentage", 2)
        self.trafo_mode = self.setupParameter("~trafo_mode", 'both')
        if not (self.trafo_mode == "cb" or self.trafo_mode == "lin" or self.trafo_mode == "both"):
            rospy.loginfo("cannot understand argument 'trafo_mode'. set to 'both' ")
            self.trafo_mode == "both"
            rospy.set_param("~trafo_mode", "both")  # Write to parameter server for transparancy
            rospy.loginfo("[%s] %s = %s " % (self.node_name, "~trafo_mode", "both"))


        # Initialize health message
        self.health = AntiInstagramHealth()

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it

        self.transform_CB = AntiInstagramTransform_CB()

        self.ai = AntiInstagram()
        # milansc: resize is done on input image below
        self.ai.setupKM(self.n_centers, self.blur, 1, self.blur_kernel)
        self.bridge = CvBridge()

        self.image_msg = None

        # timer for continuous image process
        self.timer = rospy.Timer(rospy.Duration(self.interval), self.processImage)

        # TODO write to file within git folder
        #self.file = open('/home/milan/output_cont_ai_node.txt', 'a+')
        #self.file.write('\nCONT_ANTI_INSTAGRAM_NODE:\n')

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)#Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbNewImage(self, image_msg):
        # memorize image
        self.image_msg = image_msg


    def processImage(self, event):
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

            # resize input image
            resized_img = cv2.resize(cv_image, (0, 0), fx=self.resize, fy=self.resize)
            tk.completed('resized')


            if self.trafo_mode == "cb" or self.trafo_mode == "both":
                # find color balance thresholds
                self.ai.calculateColorBalanceThreshold(resized_img, self.cb_percentage)
                tk.completed('calculateColorBalanceThresholds')

                # store color balance thresholds to ros message
                self.transform_CB.th[0], self.transform_CB.th[1], self.transform_CB.th[2] = self.ai.ThLow
                self.transform_CB.th[3], self.transform_CB.th[4], self.transform_CB.th[5] = self.ai.ThHi

                # publish color balance thresholds
                self.pub_trafo_CB.publish(self.transform_CB)
                rospy.loginfo('ai: Color balance thresholds published.')



            if self.trafo_mode == "lin" or self.trafo_mode == "both":
                if self.trafo_mode == "both":
                    # apply color balance
                    colorBalanced_image = self.ai.applyColorBalance(resized_img, self.ai.ThLow, self.ai.ThHi)
                else:
                    colorBalanced_image = resized_img


                # find color transform
                self.ai.calculateTransform(colorBalanced_image)
                tk.completed('calculateTransform')

                # store color transform to ros message
                self.transform.s[0], self.transform.s[1], self.transform.s[2] = self.ai.shift
                self.transform.s[3], self.transform.s[4], self.transform.s[5] = self.ai.scale

                # publish color trafo
                self.pub_trafo.publish(self.transform)
                rospy.loginfo('ai: Color transform published.')



                # TODO health mesurement



if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('cont_anti_instagram_node', anonymous=False)

    # Create the NodeName object
    node = ContAntiInstagramNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
