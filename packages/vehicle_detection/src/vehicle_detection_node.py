#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, Image

from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32


class VehicleDetectionNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(VehicleDetectionNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~process_frequency'] = None
        self.parameters['~circlepattern_dims'] = None
        self.parameters['~blobdetector_min_area'] = None
        self.parameters['~blobdetector_min_dist_between_blobs'] = None
        self.updateParameters()


        self.bridge = CvBridge()

        self.last_stamp = rospy.Time.now()

        # Subscriber
        self.sub_image = self.subscriber("~image", CompressedImage, self.cbImage, queue_size=1)

        # Publishers
        self.pub_detection = self.publisher("~detection", BoolStamped, queue_size=1)
        self.pub_centers = self.publisher("~centers", VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = self.publisher("~debug/detection_image/compressed", CompressedImage, queue_size=1)

        self.log("Initialization completed.")

    def cbParametersChanged(self):

        self.publish_duration = rospy.Duration.from_sec(1.0/self.parameters['~process_frequency'])
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.parameters['~blobdetector_min_area']
        params.minDistBetweenBlobs = self.parameters['~blobdetector_min_dist_between_blobs']
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(params)

    def cbImage(self, image_msg):
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now
        
        vehicle_detected_msg_out = BoolStamped()
        vehicle_centers_msg_out = VehicleCorners()
        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

        (detection, centers) = cv2.findCirclesGrid(image_cv,
                                                   patternSize=tuple(self.parameters['~circlepattern_dims']),
                                                   flags=cv2.CALIB_CB_SYMMETRIC_GRID,
                                                   blobDetector=self.simple_blob_detector)

        # if the pattern is detected, cv2.findCirclesGrid returns a non-zero result, otherwise it returns 0
        vehicle_detected_msg_out.data = detection > 0
        self.pub_detection.publish(vehicle_detected_msg_out)

        #print(centers)

        if detection:
            # print(centers)
            points_list = []
            for point in centers:
                center = Point32()
                # print(point[0])
                center.x = point[0, 0]
                # print(point[0,1])
                center.y = point[0, 1]
                center.z = 0
                points_list.append(center)
            vehicle_centers_msg_out.header.stamp = now
            vehicle_centers_msg_out.corners = points_list
            vehicle_centers_msg_out.detection.data = detection
            vehicle_centers_msg_out.H = self.parameters['~circlepattern_dims'][1]
            vehicle_centers_msg_out.W = self.parameters['~circlepattern_dims'][0]
            self.pub_centers.publish(vehicle_centers_msg_out)
        
        if True:
            cv2.drawChessboardCorners(image_cv,
                                        tuple(self.parameters['~circlepattern_dims']), centers, detection)
            image_msg_out = self.bridge.cv2_to_compressed_imgmsg(image_cv)
            self.pub_circlepattern_image.publish(image_msg_out)

if __name__ == '__main__':
    rospy.init_node('vehicle_detection', anonymous=False)
    vehicle_detection_node = VehicleDetectionNode('vehicle_detection')
    rospy.spin()
