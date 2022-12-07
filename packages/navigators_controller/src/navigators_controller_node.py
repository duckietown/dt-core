#!/usr/bin/env python3


import rospy
from duckietown.dtros import DTROS, NodeType, TopicType



class NavigatorsControllerNode(DTROS):
    
    def __init__(self, node_name):
        super(NavigatorsControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
    


if __name__ == "__main__":
    navigators_controller_node = NavigatorsControllerNode(node_name="navigators_controller_node")
    rospy.spin()