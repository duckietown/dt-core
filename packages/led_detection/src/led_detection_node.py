#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class LedDetectionNode(DTROS):
    
    def __init__(self, node_name):
        super(LedDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    led_detection_node = LedDetectionNode(node_name="led_detection_node")
    rospy.spin()