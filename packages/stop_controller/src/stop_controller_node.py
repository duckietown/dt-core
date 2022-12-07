#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class StopControllerNode(DTROS):
    
    def __init__(self, node_name):
        super(StopControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    stop_controller_node = StopControllerNode(node_name="stop_controller_node")
    rospy.spin()