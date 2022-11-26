#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class ObjectDetectorSubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ObjectDetectorSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('chatter', String, self.callback)

    def callback(self, data):
        rospy.loginfo(f"[obj-detector-sub] I heard {data.data}")

if __name__ == '__main__':
    # create the node
    node = ObjectDetectorSubscriberNode(node_name='object_detector_subscriber_node')
    # keep spinning
    rospy.spin()