#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class LineDetectionNode(DTROS):
    
    def __init__(self, node_name):
        super(LineDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    line_detection_node = LineDetectionNode(node_name="line_detection_node")
    rospy.spin()