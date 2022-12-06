#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class ObstacleDetectionNode(DTROS):
    
    def __init__(self, node_name):
        super(ObstacleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    obstacle_detection_node = ObstacleDetectionNode(node_name="obstacle_detection_node")
    rospy.spin()