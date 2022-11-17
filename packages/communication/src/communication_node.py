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

        self.buffer = ImgBuffer(200)

    def run(self):
        pass

    def callback(self, data):
        new_img = cv2.cvtColor(np.frombuffer(data.data, np.uint8), cv2.COLOR_RGB2GRAY)
        self.buffer.push(new_img)

        if len(self.buffer.diff_imgs) < 2:
            return

        sub_images = [(get_sub(self.buffer.raw_imgs, point, 10), point) for point in self.buffer.points]
        frequencies = [(get_frequency(sub), point) for sub, point in sub_images]
        rospy.loginfo(f"frame {frequencies}")


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    node.run()
    # keep spinning
    rospy.spin()
