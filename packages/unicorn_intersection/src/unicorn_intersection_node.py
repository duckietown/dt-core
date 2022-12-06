#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class UnicornIntersectionNode(DTROS):
    
    def __init__(self, node_name):
        super(UnicornIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.PLANNING)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    unicorn_intersection_node = UnicornIntersectionNode(node_name="unicorn_intersection_node")
    rospy.spin()