#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class AntiInstagramNode(DTROS):
    
    def __init__(self, node_name):
        # print("lane_filter_node")
        super(AntiInstagramNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    



if __name__ == "__main__":
    anti_instagram_node = AntiInstagramNode(node_name="anti_instagram_node")
    rospy.spin()