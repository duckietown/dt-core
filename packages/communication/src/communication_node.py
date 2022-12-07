#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class CommunicationNode(DTROS):
    
    def __init__(self, node_name):
        super(CommunicationNode, self).__init__(node_name=node_name, node_type=NodeType.COMMUNICATION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    



if __name__ == "__main__":
    communication_node = CommunicationNode(node_name="communication_node")
    rospy.spin()