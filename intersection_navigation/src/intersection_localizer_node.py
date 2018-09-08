#!/usr/bin/env python
import rospy
import cv2
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from duckietown_msgs.msg import IntersectionPoseImg, IntersectionPose, IntersectionPoseImgDebug, FSMState
import numpy as np


class IntersectionLocalizerNode(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up path planner, state estimator, ...
        self.intersectionLocalizer = IntersectionLocalizer(self.veh)
        self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')

        # types
        self.intersection_types = {0: 'THREE_WAY_INTERSECTION', 1: 'FOUR_WAY_INTERSECTION'}

        # initializing variables
        self.active = True

        self.sub_pose_img = rospy.Subscriber("~pose_img_in",
                                        IntersectionPoseImg,
                                        self.PoseImageCallback,
                                        queue_size=1, buff_size=2**24)

        # set up publishers
        self.pub_pose = rospy.Publisher("~pose_out", IntersectionPose, queue_size=1)
        self.pub_debug = rospy.Publisher("~localizer_debug_out", IntersectionPoseImgDebug, queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def PoseImageCallback(self, msg):


        if not self.active:
            return
        # set intersection type
        self.intersectionLocalizer.SetEdgeModel(self.intersection_types[msg.type])

        pose_pred = np.array([msg.x, msg.y, msg.theta])
        img_processed, _ = self.intersectionLocalizer.ProcessRawImage(msg.img)
        valid_meas, pose_meas, likelihood = self.intersectionLocalizer.ComputePose(img_processed, pose_pred)

        # update pose estimate
        if valid_meas:
            msg_ret = IntersectionPose()
            msg_ret.header.stamp = msg.header.stamp
            msg_ret.x = pose_meas[0]
            msg_ret.y = pose_meas[1]
            msg_ret.theta = pose_meas[2]
            msg_ret.type = msg.type
            msg_ret.likelihood = likelihood
            self.pub_pose.publish(msg_ret)

            if 1:
                msg_debug = IntersectionPoseImgDebug()
                msg_debug.header.stamp = rospy.Time.now()
                msg_debug.x = pose_meas[0]
                msg_debug.y = pose_meas[1]
                msg_debug.theta = pose_meas[2]
                msg_debug.type = msg.type
                msg_debug.likelihood = likelihood
                msg_debug.x_init = pose_pred[0]
                msg_debug.y_init = pose_pred[1]
                msg_debug.theta_init = pose_pred[2]
                msg_debug.img = msg.img
                self.pub_debug.publish(msg_debug)

            # sleep for one image (to make sure estimator gets updates before processing next image)
            rospy.sleep(0.034)




    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value




    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('intersection_localizer_node', anonymous=False)

    # create the intersection navigation object
    node = IntersectionLocalizerNode()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    rospy.spin()
