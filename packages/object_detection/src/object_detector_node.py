#!/usr/bin/env python3

import os
import cv2
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from dt_device_utils import DeviceHardwareBrand, get_device_hardware_brand

class ObjectDetectorNode(DTROS):

    def __init__(self, node_name):
        #* initialize the DTROS parent class
        super(ObjectDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        #* construct publishers 
        self.pub = rospy.Publisher('chatter', String, queue_size=1) # Todo: Remove when not needed anymore
        self. pub_detections_image = rospy.Publisher("~object_detections_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG)
        # Todo: Create publisher to publish detection and position

        #* Subscribe to the image
        self.sub_img = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.callback,
            buff_size=10000000,
            queue_size=1
        )

        #* Params
        self.bridge = CvBridge()
        # Todo: Load model
        self.initialized = True
    

    def callback(self, img_msg):
        if not self.initialized:
            return

        rospy.loginfo("Img receive for processing")
        
        #* img: ROS -> OpenCV 
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        except ValueError as e:
            self.logerr(f"[obj-detector] Could not convert img_msg to img: {e}")
            return

        #* Resize img and color
        old_img = image
        #* Handle sim
        if get_device_hardware_brand() != DeviceHardwareBrand.JETSON_NANO:  
            image = image[...,::-1].copy()  # image is bgr, flip it to rgb
        old_img = cv2.resize(old_img, (416,416))
        image = cv2.resize(image, (416,416))

        # Todo: Do prediction from the img
        # ? Add a simple model if Debug is True using only the yellow color

        # Todo: Find position of the objects detected

        # Todo: Publish detection


        #* Publish detection img for demo
        # Todo: Add bounding box
        obj_det_img = self.bridge.cv2_to_imgmsg(old_img, encoding="bgr8")
        self.pub_detections_image.publish(obj_det_img)


    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            message = f"{os.environ['VEHICLE_NAME']} is on the moon!"
            rospy.loginfo(f"[obj-detector] Publishing message: {message}")
            self.pub.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ObjectDetectorNode(node_name='object_detection_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()