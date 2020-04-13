#!/usr/bin/env python


import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from image_geometry import PinholeCameraModel

from duckietown import DTROS
from duckietown_msgs.msg import BoolStamped, VehicleCorners
# from geometry_msgs.msg import Point32

from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose, Pose2DStamped, StopLineReading
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
# from math import sqrt, sin, cos
# from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import yaml


class VehicleFilterNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(VehicleFilterNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~distance_between_centers'] = None
        self.parameters['~max_reproj_pixelerror_pose_estimation'] = None
        self.parameters['~virtual_stop_line_offset'] = None
        self.updateParameters()

        self.bridge = CvBridge()

        # these will be defined on the first call to calc_circle_pattern
        self.last_calc_circle_pattern = None
        self.circlepattern_dist = None
        self.circlepattern = None

        # subscribers
        self.sub_centers = self.subscriber("~centers", VehicleCorners, self.cb_process_centers, queue_size=1)
        self.sub_info = self.subscriber("~camera_info", CameraInfo, self.cb_process_camera_info, queue_size=1)

        # publishers
        # self.pub_pose = self.publisher("~pose", VehiclePose, queue_size=1)
        self.pub_virtual_stop_line = self.publisher("~virtual_stop_line", StopLineReading, queue_size=1)
        self.pub_visualize = self.publisher("~debug/visualization_marker", Marker, queue_size=1)

        self.pcm = PinholeCameraModel()
        self.log("Initialization completed")

    def cb_process_camera_info(self, camera_info_msg):
        with self.phasetimer.time_phase('cb_process_camera_info callback'):
            self.pcm.fromCameraInfo(camera_info_msg)

    def cb_process_centers(self, vehicle_centers_msg):

        # check if there actually was a detection
        detection = vehicle_centers_msg.detection.data

        if detection:
            with self.phasetimer.time_phase('solve PnP'):
                self.calc_circle_pattern(vehicle_centers_msg.H, vehicle_centers_msg.W)
                points = np.zeros((vehicle_centers_msg.H * vehicle_centers_msg.W, 2))
                for i in range(len(points)):
                    points[i] = np.array([vehicle_centers_msg.corners[i].x, vehicle_centers_msg.corners[i].y])

                success, rotation_vector, translation_vector = cv2.solvePnP(objectPoints=self.circlepattern,
                                                                            imagePoints=points,
                                                                            cameraMatrix=self.pcm.intrinsicMatrix(),
                                                                            distCoeffs=self.pcm.distortionCoeffs())

            if success:
                with self.phasetimer.time_phase('project points and calculate reproj. error'):
                    points_reproj, _ = cv2.projectPoints(objectPoints=self.circlepattern,
                                                         rvec=rotation_vector,
                                                         tvec=translation_vector,
                                                         cameraMatrix=self.pcm.intrinsicMatrix(),
                                                         distCoeffs=self.pcm.distortionCoeffs())

                mean_reproj_error = np.mean(np.sqrt(np.sum((np.squeeze(points_reproj)-points)**2, axis=1)))

                if mean_reproj_error < self.parameters['~max_reproj_pixelerror_pose_estimation']:
                    with self.phasetimer.time_phase('calculate pose and publish'):
                        (R, jac) = cv2.Rodrigues(rotation_vector)
                        R_inv = np.transpose(R)
                        translation_vector = -np.dot(R_inv, translation_vector)
                        distance_to_vehicle = -translation_vector[2]
                        print("TRANSLATION VECTOR: ", translation_vector)
                        # make the message and publish
                        stop_line_msg = StopLineReading()
                        stop_line_msg.header = vehicle_centers_msg.header
                        stop_line_msg.stop_line_detected = True
                        stop_line_msg.at_stop_line = distance_to_vehicle <= self.parameters['~virtual_stop_line_offset']
                        stop_line_msg.stop_line_point.x = distance_to_vehicle
                        self.pub_virtual_stop_line.publish(stop_line_msg)

                    if self.pub_visualize.get_num_connections() > 0:
                        with self.phasetimer.time_phase('publish marker'):
                            marker_msg = Marker()
                            marker_msg.header = vehicle_centers_msg.header
                            marker_msg.header.frame_id = 'donald'
                            marker_msg.ns = 'my_namespace'
                            marker_msg.id = 0
                            marker_msg.type = Marker.CUBE
                            marker_msg.action = Marker.ADD
                            marker_msg.pose.position.x = -translation_vector[2]
                            marker_msg.pose.position.y = translation_vector[0]
                            marker_msg.pose.position.z = translation_vector[1]
                            marker_msg.pose.orientation.x = 0.0
                            marker_msg.pose.orientation.y = 0.0
                            marker_msg.pose.orientation.z = 0.0
                            marker_msg.pose.orientation.w = 1.0
                            marker_msg.scale.x = 0.1
                            marker_msg.scale.y = 0.1
                            marker_msg.scale.z = 0.1
                            marker_msg.color.r = 1.0
                            marker_msg.color.g = 1.0
                            marker_msg.color.b = 0.0
                            marker_msg.color.a = 1.0
                            marker_msg.lifetime = rospy.Duration.from_sec(1)
                            self.pub_visualize.publish(marker_msg)

                else:
                    self.log("Pose estimation failed, too high reprojection error. "
                             "Reporting detection at 0cm for safety.")
                    stop_line_msg = StopLineReading()
                    stop_line_msg.header = vehicle_centers_msg.header
                    stop_line_msg.stop_line_detected = True
                    stop_line_msg.at_stop_line = True
                    stop_line_msg.stop_line_point.x = 0
                    stop_line_msg.stop_line_point.y = 0
                    self.pub_virtual_stop_line.publish(stop_line_msg)

            else:
                self.log("Pose estimation failed. "
                         "Reporting detection at 0cm for safety.")
                stop_line_msg = StopLineReading()
                stop_line_msg.header = vehicle_centers_msg.header
                stop_line_msg.stop_line_detected = True
                stop_line_msg.at_stop_line = True
                stop_line_msg.stop_line_point.x = 0
                stop_line_msg.stop_line_point.y = 0
                self.pub_virtual_stop_line.publish(stop_line_msg)

        else:
            # publish empty messages
            stop_line_msg = StopLineReading()
            stop_line_msg.header = vehicle_centers_msg.header
            stop_line_msg.stop_line_detected = False
            stop_line_msg.at_stop_line = False
            self.pub_virtual_stop_line.publish(stop_line_msg)

        if self.pub_visualize.get_num_connections() > 0:
            marker_msg = Marker()
            marker_msg.header = vehicle_centers_msg.header
            marker_msg.header.frame_id = 'donald'
            marker_msg.ns = 'my_namespace'
            marker_msg.id = 0
            marker_msg.action = Marker.DELETE
            self.pub_visualize.publish(marker_msg)

    def calc_circle_pattern(self, height, width):
        # check if the version generated before is still valid, if not, or first time called, create

        with self.phasetimer.time_phase('calc_circle_pattern callback'):
            if self.last_calc_circle_pattern is None or self.last_calc_circle_pattern != (height, width):
                self.circlepattern_dist = self.parameters['~distance_between_centers']
                self.circlepattern = np.zeros([height * width, 3])
                for i in range(0, width):
                    for j in range(0, height):
                        self.circlepattern[i + j * width, 0] = self.circlepattern_dist * i - \
                                                               self.circlepattern_dist * (width - 1) / 2
                        self.circlepattern[i + j * width, 1] = self.circlepattern_dist * j - \
                                                               self.circlepattern_dist * (height - 1) / 2


if __name__ == '__main__':
    vehicle_filter_node = VehicleFilterNode(node_name='vehicle_filter_node')
    rospy.spin()
