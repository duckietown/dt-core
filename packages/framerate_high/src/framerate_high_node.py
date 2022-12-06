#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class FramerateHighNode(DTROS):
    
    def __init__(self, node_name):
        super(FramerateHighNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    



if __name__ == "__main__":
    framerate_high_node = FramerateHighNode(node_name="framerate_high_node")
    rospy.spin()