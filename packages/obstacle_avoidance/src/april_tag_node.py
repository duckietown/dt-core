#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType

class AprilTagNode(DTROS):

    def __init__(self, node_name):
        print("april_tag_node")
        super(AprilTagNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)


if __name__ == "__main__":
    april_tag_node = AprilTagNode(node_name="april_tag_node")
    rospy.spin()
