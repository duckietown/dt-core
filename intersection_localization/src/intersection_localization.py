#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import IntersectionPose, BoolStamped, FSMState
import numpy as np
import duckietown_utils as dt

from intersection_localizer.intersection_localizer import IntersectionLocalizer


class IntersectionLocalization(object):
    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        self.active = True
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up subscribers
        # self.sub_mode = rospy.Subscriber("~mode", FSMState, self.ModeCallback, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.SwitchCallback, queue_size=1)
        self.sub_img = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed", CompressedImage,
                                        self.ImageCallback, queue_size=1)

        # set up publishers
        self.pub_intersection_pose_meas = rospy.Publisher("~intersection_pose_meas", IntersectionPose, queue_size=1)

        # set up localizer
        self.localizer = IntersectionLocalizer(self.veh)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

        # temp
        self.at_intersection = False
        self.init = 0


    def ImageCallback(self, msg_img):
        if self.at_intersection:
            # process raw image
            img_processed, img_gray = self.localizer.ProcessRawImage(msg_img)

            # get pose estimation
            '''msg_pose_pred = rospy.wait_for_message('~intersection_pose_pred', IntersectionPose)'''
            # TODO: also add type of intersection in above message!

            # temp
            if not self.init:
                self.init = 0
                x_pred = 0.415
                y_pred = -0.16
                theta_pred = np.pi / 2.0

            else:
                x_pred = self.x_meas
                y_pred = self.y_meas
                theta_pred = self.theta_meas

            # compute the Duckiebot's pose
            valid_meas, x_meas, y_meas, theta_meas = self.localizer.ComputePose(img_processed, x_pred, y_pred, theta_pred, 'THREE_WAY_INTERSECTION')

            if valid_meas:
                # publish results
                msg_pose_meas = IntersectionPose()
                msg_pose_meas.header.stamp = msg_img.header.stamp
                msg_pose_meas.x = x_meas
                msg_pose_meas.y = y_meas
                msg_pose_meas.theta = theta_meas
                self.pub_intersection_pose_meas.publish(msg_pose_meas)

            # debugging
            if 1:
                self.x_meas = x_meas
                self.y_meas = y_meas
                self.theta_meas = theta_meas
                self.localizer.DrawModel(img_gray, x_meas, y_meas, theta_meas, 'THREE_WAY_INTERSECTION')
                cv2.imshow('img_prediction', img_gray)
                cv2.waitKey(1)

        else:
            return

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))
        cv2.destroyAllWindows()

    def SwitchCallback(self, msg):
        self.at_intersection = msg.data #True or False
        rospy.loginfo("active: " + str(self.active))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('intersection_localization_node', anonymous=False)

    # Create the NodeName object
    node = IntersectionLocalization()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
