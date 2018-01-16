#!/usr/bin/env python
import rospy
from anti_instagram.AntiInstagram_rebuild import *
from cv_bridge import CvBridge  # @UnresolvedImport
# @UnresolvedImport
from duckietown_msgs.msg import (AntiInstagramHealth, AntiInstagramTransform, AntiInstagramTransform_CB, BoolStamped)
from duckietown_utils.jpg import image_cv_from_jpg
from line_detector.timekeeper import TimeKeeper
from sensor_msgs.msg import CompressedImage, Image  # @UnresolvedImport
import numpy as np

import time
import threading

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
        self.active = True
        self.locked = False
        self.thread_lock = threading.Lock()
        self.r = rospy.Rate(4) # Rate in Hz
        robot_name = rospy.get_param("~veh", "") #to read the name always reliably
        cont_mode = rospy.get_param("~cont_mode", "") #tells which topic to listen to
        cont_mode = False

        # Initialize publishers and subscribers
        self.pub_image = rospy.Publisher(
            "~corrected_image", Image, queue_size=1)

        self.sub_image = rospy.Subscriber(
            # "/duckierick/image_transformer_node/uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)
            # "~uncorrected_image", CompressedImage, self.cbNewImage, queue_size=1)
            '/{}/camera_node/image/compressed'.format(robot_name), CompressedImage, self.callbackImage, queue_size=1)

        if (cont_mode):
            self.sub_trafo = rospy.Subscriber(
                '/{}/cont_anti_instagram_node/transform'.format(robot_name), AntiInstagramTransform, self.cbNewTrafo, queue_size=1)
                # "/duckierick/cont_anti_instagram_node/transform", AntiInstagramTransform, self.cbNewTrafo, queue_size = 1)
        else:
            print "HERE"
            self.sub_trafo = rospy.Subscriber(
                '/{}/anti_instagram_node/transform'.format(robot_name), AntiInstagramTransform, self.cbNewTrafo, queue_size=1)
                # "/duckierick/cont_anti_instagram_node/transform", AntiInstagramTransform, self.cbNewTrafo, queue_size = 1)

        self.sub_colorBalance = rospy.Subscriber(
            '/{}/cont_anti_instagram_node/colorBalanceTrafo'.format(robot_name), AntiInstagramTransform_CB, self.cbNewTrafo_CB, queue_size=1)

        # Read parameters
        self.trafo_mode = self.setupParameter("~trafo_mode", 'both')
        if not (self.trafo_mode == "cb" or self.trafo_mode == "lin" or self.trafo_mode == "both"):
            rospy.loginfo("cannot understand argument 'trafo_mode'. set to 'both' ")
            self.trafo_mode == "both"
            rospy.set_param("~trafo_mode", "both")  # Write to parameter server for transparancy
            rospy.loginfo("[%s] %s = %s " % (self.node_name, "~trafo_mode", "both"))

        if not(cont_mode):
            self.trafo_mode = "lin"

        # TODO verify name of parameter
        # Verbose option
        self.verbose = rospy.get_param('line_detector_node/verbose', False)

        # Initialize transform message
        self.transform = AntiInstagramTransform()
        # FIXME: read default from configuration and publish it
        self.transform_CB = AntiInstagramTransform_CB()

        self.corrected_image = Image()
        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        self.image_msg = None


    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)#Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def callbackImage(self, image_msg):
        #use this to avoid lag!
        thread = threading.Thread(target=self.cbNewImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def cbNewImage(self, image_msg):

        if not self.thread_lock.acquire(False):
                return

        #start = time.time() #with this you can measure the time,..
        # print('image received!')
        # memorize image
        self.image_msg = image_msg

        tk = TimeKeeper(image_msg)
        #cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        try:
            cv_image = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
            return

        tk.completed('converted')


        if self.trafo_mode == "cb" or self.trafo_mode == "both":

            # apply color balance using latest thresholds
            colorBalanced_image_cv2 = self.ai.applyColorBalance(img=cv_image, ThLow=self.ai.ThLow, ThHi=self.ai.ThHi)
            tk.completed('applyColorBalance')
        else:
            colorBalanced_image_cv2 = cv_image

        if self.trafo_mode == "lin" or self.trafo_mode == "both":
            # apply color Transform using latest parameters
            corrected_image_cv2 = self.ai.applyTransform(colorBalanced_image_cv2)
            tk.completed('applyTransform')
        else:
            corrected_image_cv2 = colorBalanced_image_cv2

        # store image to ros message
        self.corrected_image = self.bridge.cv2_to_imgmsg(
            corrected_image_cv2, "bgr8")
        tk.completed('encode')

        self.corrected_image.header.stamp = image_msg.header.stamp #for synchronization

        self.pub_image.publish(self.corrected_image)
        tk.completed('published')

        #end = time.time()   #with this you can measure the time,..
        #print "TRANSFORMING IMAGE TOOK: s"  #with this you can measure the time,..
        #print(end - start)  #with this you can measure the time,..

        if self.verbose:
            rospy.loginfo('ai:\n' + tk.getall())

        self.r.sleep() #to keep the rate
        self.thread_lock.release()


    def cbNewTrafo(self, trafo_msg):
        # print('image transformer: received new trafo!')
        # testwise write to file
        # self.file.write('received new trafo\n')

        if self.verbose:
            rospy.loginfo('image transformer: received new trafo!')

        # memorize transform message
        self.transform = trafo_msg

        # store transform to the Anti-Instagram instance
        self.ai.shift = trafo_msg.s[0:3]         #copied from line_detector2 ldn.py
        self.ai.scale = trafo_msg.s[3:6]

        if (abs(self.ai.shift[0]-0)>0.2): #this tells whether we can start or not!!!
            rospy.logwarn('!!!!!!!!!!!!!!!!!!!!TRAFO WAS COMPUTED SO WE ARE READY TO GO!!!!!!!!!!!!')

    def cbNewTrafo_CB(self, th_msg):
        # print('image transformer: received new Color Balance trafo!')
        if self.verbose:
            rospy.loginfo('image transformer: received new Color Balance trafo!')

        self.transform_CB = th_msg
        self.ai.ThLow = th_msg.th[0:3]
        self.ai.ThHi = th_msg.th[3:6]




if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('image_transformer_node', anonymous=False)

    # Create the NodeName object
    node = ImageTransformerNode()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
