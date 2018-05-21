#!/usr/bin/env python
import rospy
import cv2
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import IntersectionPoseImgDebug
from duckietown_utils import robot_name
import numpy as np


class IntersectionVisualizer(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up intersection localizer
        self.intersection_types = {0: 'THREE_WAY_INTERSECTION', 1: 'FOUR_WAY_INTERSECTION'}
        self.intersectionLocalizer = IntersectionLocalizer(self.veh)
        rospy.loginfo ("Veh is set on: " +str(self.veh))
        self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')

        self.sub_img = rospy.Subscriber("~img",
                                        IntersectionPoseImgDebug,
                                        self.ImageCallback,
                                        queue_size=1)
        self.pose = np.array([0.400, -0.105, 0.5 * np.pi])

        rospy.loginfo("[%s] Initialized." % (self.node_name))

        self.debug_init = False
        self.k = 0


    def ImageCallback(self, msg):
        if not self.debug_init:
            self.debug_init = True
            self.start_time = msg.img.header.stamp

        _, img_gray = self.intersectionLocalizer.ProcessRawImage(msg.img)
        pose_pred = np.array([msg.x_init,msg.y_init, msg.theta_init])
        pose_meas = np.array([msg.x,msg.y, msg.theta])
        self.intersectionLocalizer.SetEdgeModel(self.intersection_types[msg.type])
        self.intersectionLocalizer.DrawModel(img_gray, pose_pred, True)
        self.intersectionLocalizer.DrawModel(img_gray, pose_meas)

        cv2.imshow('Estimate' + str(self.k) , img_gray)
        cv2.waitKey(50)

        print('-------------------')
        print('k', self.k)
        print('absolute time', (msg.header.stamp - self.start_time).to_sec())
        print('proc time', (msg.header.stamp - msg.img.header.stamp).to_sec())
        print('likelihood', msg.likelihood)

        self.k += 1


    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def OnShutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('intersection_visualizer_node', anonymous=False)

    # create the intersection navigation object
    node = IntersectionVisualizer()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # spin to prevent stopping
    rospy.spin()
