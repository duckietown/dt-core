#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, TopicType, NodeType


class LEDEmitterNode(DTROS):


    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDEmitterNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)


if __name__ == "__main__":
    # Create the LEDEmitterNode object
    led_emitter_node = LEDEmitterNode(node_name="led_emitter")
    # Keep it spinning to keep the node alive
    rospy.spin()
