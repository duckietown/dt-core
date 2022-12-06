#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class ObjectDetectionNode(DTROS):
    
    def __init__(self, node_name):
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
    rospy.spin()