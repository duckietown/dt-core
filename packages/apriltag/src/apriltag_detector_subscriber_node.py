#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import AprilTagsWithInfos
from std_msgs.msg import String

class ApriltagDetectorSubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ApriltagDetectorSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.sub = rospy.Subscriber(f"apriltag_postprocessing_node/apriltags_out", AprilTagsWithInfos, self.callback, queue_size=1)

    def callback(self, data):
        rospy.loginfo(f"DETECT APRILTAGS {data}")

if __name__ == '__main__':
    # create the node
    node = ApriltagDetectorSubscriberNode(node_name='apriltag_detector_subscriber_node')

    # keep spinning
    rospy.spin()