#!/usr/bin/env python3

import rospy
from BaseComNode import BaseComNode
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest


class MySubscriberNode(DTROS, BaseComNode):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        DTROS.__init__(self, node_name=node_name, node_type=NodeType.GENERIC)
        BaseComNode.__init__(self, 60, 40)
        # Subscribe to needed inputs & services
        self.sub = rospy.Subscriber('camera_node/image/compressed', CompressedImage, self.img_callback)
        self.serv = rospy.ServiceProxy('led_emitter_node/set_custom_pattern', SetCustomLEDPattern)

    def blink_at(self, frequency: int):
        self.serv(SetCustomLEDPatternRequest(pattern=LEDPattern(
            frequency=frequency,
            color_list=['white'] * 5,
            color_mask=[1] * 5,
            frequency_mask=[1] * 5,
        )))

    def run(self):
        rate = rospy.Rate(0.5)  # 1Hz
        while not rospy.is_shutdown():
            # TODO: fill
            action = BaseComNode.run(self)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    node.run()
    # keep spinning
    rospy.spin()
