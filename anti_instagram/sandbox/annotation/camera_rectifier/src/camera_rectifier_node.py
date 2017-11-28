#!/usr/bin/env python
import rospy
from camera_image_rectifier import CameraImageRectifier
from duckietown_utils import get_base_name, rgb_from_ros
from sensor_msgs.msg import CompressedImage


class CameraRectifierNode(object):
    def __init__(self):
        self.node_name = "Camera Rectifier Node"
        robot_name = rospy.get_param("~robot_name")

        self.image_rectifier = CameraImageRectifier(robot_name)

        # Create a Publisher
        self.pub_topic = '/{}/camera_rectifier/image/compressed'.format(
            robot_name)
        self.publisher = rospy.Publisher(
            self.pub_topic, CompressedImage, queue_size=1)

        # Create a Subscriber
        self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
        self.subscriber = rospy.Subscriber(
            self.sub_topic, CompressedImage, self.callback)

    def callback(self, image):
        rectified_img = CompressedImage()
        rectified_img.header.stamp = image.header.stamp
        rectified_img.format = "jpeg"
        rectified_img.data = self.image_rectifier.process_image(
            rgb_from_ros(image))

        # publish new message
        self.publisher.publish(rectified_img)


if __name__ == '__main__':
    rospy.init_node('camera_rectifier_node', anonymous=False)
    camera_rectifier_node = CameraRectifierNode()
    rospy.spin()
