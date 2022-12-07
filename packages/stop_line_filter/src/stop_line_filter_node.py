#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class StopLineFilterNode(DTROS):
    
    def __init__(self, node_name):
        super(StopLineFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    stop_line_filter_node = StopLineFilterNode(node_name="stop_line_filter_node")
    rospy.spin()