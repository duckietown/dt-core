#!/usr/bin/env python3

import os
import numpy as np
import cv2
import rospy
from img_analysis import get_sub, get_frequency
from ImgBuffer import ImgBuffer
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest


class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Subscribe to needed inputs & services
        self.sub = rospy.Subscriber('camera_node/image/compressed', CompressedImage, self.callback)
        self.serv = rospy.ServiceProxy('led_emitter_node/set_custom_pattern', SetCustomLEDPattern)

        self.buffer = ImgBuffer(60, 40)

    def run(self):
        pass

    def callback(self, data):
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.buffer.push(new_img)

        for i, point in enumerate(self.buffer.points):
            print(f"{i} -- freq: {point.get_frequency()[0]} -- {point}")


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    node.run()
    # keep spinning
    rospy.spin()
