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
from duckietown_msgs.msg import VehicleCorners, VehiclePose, Pose2DStamped
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
        self.pub_pose = self.publisher("~pose", VehiclePose, queue_size=1)
        self.pub_visualize = self.publisher("~debug/visualization_marker", Marker, queue_size=1)

        self.pcm = PinholeCameraModel()
        self.log("Initialization completed")

    def cb_process_camera_info(self, camera_info_msg):
        with self.phasetimer.time_phase('cb_process_camera_info callback'):
            self.pcm.fromCameraInfo(camera_info_msg)

    def cb_process_centers(self, vehicle_centers_msg):
        self.calc_circle_pattern(vehicle_centers_msg.H, vehicle_centers_msg.W)
	dist_coeffs = np.array([self.pcm.distortionCoeffs()[0][0,0], self.pcm.distortionCoeffs()[1][0,0], self.pcm.distortionCoeffs()[2][0,0], self.pcm.distortionCoeffs()[3][0,0], self.pcm.distortionCoeffs()[4][0,0]])
        with self.phasetimer.time_phase('solve PnP'):
            points = np.zeros((vehicle_centers_msg.H * vehicle_centers_msg.W, 2))
            for i in range(len(points)):
                points[i] = np.array([vehicle_centers_msg.corners[i].x, vehicle_centers_msg.corners[i].y])
                print(points[i])

	    print("DISTORTION COEFFICIENTS:", dist_coeffs) 
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
                # TODO:
                print("points_reproj.shape", points_reproj.shape)
                error = 0
                for i in range(0, len(points_reproj)):
                    error += cv2.norm(points[i],
                                      points_reproj[i, 0], cv2.NORM_L2)

                mean_reproj_error = error / len(points_reproj)

            if mean_reproj_error < self.parameters['~max_reproj_pixelerror_pose_estimation']:
                with self.phasetimer.time_phase('calculate pose and publish'):
                    (R, jac) = cv2.Rodrigues(rotation_vector)
                    R_inv = np.transpose(R)
                    translation_vector = -np.dot(R_inv, translation_vector)
                    pose_msg_out = VehiclePose()
                    pose_msg_out.header.stamp = rospy.Time.now()
                    pose_msg_out.rho.data = np.sqrt(translation_vector[2] ** 2 + translation_vector[0] ** 2)
                    pose_msg_out.psi.data = np.arctan2(-R_inv[2, 0], np.sqrt(R_inv[2, 1] ** 2 + R_inv[2, 2] ** 2))
                    pose_msg_out.detection.data = vehicle_centers_msg.detection.data
                    R2 = np.array([[np.cos(pose_msg_out.psi.data), -np.sin(pose_msg_out.psi.data)],
                                   [np.sin(pose_msg_out.psi.data), np.cos(pose_msg_out.psi.data)]])
                    translation_vector_2d = - \
                        np.array([translation_vector[2],
                                  translation_vector[0]])
                    translation_vector_2d = np.dot(
                        np.transpose(R2), translation_vector_2d)
                    pose_msg_out.theta.data = np.arctan2(
                        translation_vector_2d[1], translation_vector_2d[0])
                    self.pub_pose.publish(pose_msg_out)

                #TODO: Put subscription check
                if True:
                    with self.phasetimer.time_phase('publish marker'):
                        marker_msg = Marker()
                        marker_msg.header = pose_msg_out.header
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
                self.log("Pose estimation failed, too high reprojection error.")
        else:
            self.log("Pose estimation failed.")

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
