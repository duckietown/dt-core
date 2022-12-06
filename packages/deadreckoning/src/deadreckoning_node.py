#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class DeadreckoningNode(DTROS):
    
    def __init__(self, node_name):
        super(DeadreckoningNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    



if __name__ == "__main__":
    deadreckoning_node = DeadreckoningNode(node_name="deadreckoning_node")
    rospy.spin()