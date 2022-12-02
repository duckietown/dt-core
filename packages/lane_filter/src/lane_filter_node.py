#!/usr/bin/env python3
# import json

# import numpy as np

import rospy
# from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
# from duckietown_msgs.msg import FSMState, LanePose, SegmentList, Twist2DStamped
# from lane_filter import LaneFilterHistogram
# from sensor_msgs.msg import Image
# from std_msgs.msg import String


class LaneFilterNode(DTROS):
    
    def __init__(self, node_name):
        print("lane_filter_node")
        super(LaneFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    
    # def test_print():


if __name__ == "__main__":
    lane_filter_node = LaneFilterNode(node_name="lane_filter_node")
    rospy.spin()
    
 
