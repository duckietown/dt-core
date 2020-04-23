#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import os
import yaml
import numpy as np

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from ground_projection import Point, GroundProjectionGeometry
from image_geometry import PinholeCameraModel

from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Point as PointMsg
from duckietown_msgs.msg import Segment, SegmentList


class GroundProjectionNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(GroundProjectionNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self.bridge = CvBridge()
        self.pcm = None
        self.ground_projector = None
        self.homography = self.load_extrinsics()
        self.first_processing_done = False

        # subscribers
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber("~lineseglist_in", SegmentList, self.lineseglist_cb, queue_size=1)

        # publishers
        self.pub_lineseglist = rospy.Publisher("~lineseglist_out",
                                               SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_debug_img = rospy.Publisher("~debug/ground_projection_image/compressed",
                                             CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG)

        self.bridge = CvBridge()

        self.debug_img_bg = None

        # Seems to be never used:
        # self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography, self.estimate_homography_cb)
        # self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cb)
        # self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord, self.get_image_coordinate_cb)

    def cb_camera_info(self, msg):
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(msg)
        self.ground_projector = GroundProjectionGeometry(im_width=msg.width,
                                                         im_height=msg.height,
                                                         homography=np.array(self.homography).reshape((3, 3)))

    def pixel_msg_to_ground_msg(self, point_msg):
        # normalized coordinates to pixel:
        norm_pt = Point.from_message(point_msg)
        pixel = self.ground_projector.vector2pixel(norm_pt)
        # rectify
        rect = Point(*list(self.pcm.rectifyPoint([pixel.x, pixel.y])))
        # convert to Point
        rect_pt = Point.from_message(rect)
        # project on ground
        ground_pt = self.ground_projector.pixel2ground(rect_pt)
        # point to message
        ground_pt_msg = PointMsg()
        ground_pt_msg.x = ground_pt.x
        ground_pt_msg.y = ground_pt.y
        ground_pt_msg.z = ground_pt.z

        return ground_pt_msg

    def lineseglist_cb(self, seglist_msg):
        if self.pcm is None or self.ground_projector:
            seglist_out = SegmentList()
            seglist_out.header = seglist_msg.header
            for received_segment in seglist_msg.segments:
                new_segment = Segment()
                new_segment.points[0] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[0])
                new_segment.points[1] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[1])
                new_segment.color = received_segment.color
                # TODO what about normal and points
                seglist_out.segments.append(new_segment)
            self.pub_lineseglist.publish(seglist_out)

            if not self.first_processing_done:
                self.log('First projected segments published.')
                self.first_processing_done = True

            if self.pub_debug_img.get_num_connections() > 0:
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.debug_image(seglist_out))
                debug_image_msg.header = seglist_out.header
                self.pub_debug_img.publish(debug_image_msg)
        else:
            self.log('Waiting for a CameraInfo message', 'warn')

    # def get_ground_coordinate_cb(self, req):
    #     return GetGroundCoordResponse(self.pixel_msg_to_ground_msg(req.uv))
    #
    # def get_image_coordinate_cb(self, req):
    #     return GetImageCoordResponse(self.gpg.ground2pixel(req.gp))
    #
    # def estimate_homography_cb(self, req):
    #     rospy.loginfo("Estimating homography")
    #     rospy.loginfo("Waiting for raw image")
    #     img_msg = rospy.wait_for_message("/" + self.robot_name + "/camera_node/image/raw", Image)
    #     rospy.loginfo("Got raw image")
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #     self.gp.estimate_homography(cv_image)
    #     rospy.loginfo("wrote homography")
    #     return EstimateHomographyResponse()

    def load_extrinsics(self):
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        stream = file(cali_file, 'r')

        # TODO; catch errors
        calib_data = yaml.load(stream)

        return calib_data['homography']

    def debug_image(self, seg_list):
        # dimensions of the image are 1m x 1m so, 1px = 2.5mm
        # the origin is at x=200 and y=300

        # if that's the first call, generate the background
        if not self.debug_img_bg:

            # initialize gray image
            self.debug_img_bg = np.ones((400, 400, 3), np.uint8) * 128

            # draw vertical lines of the grid
            for vline in np.arange(40,361,40):
                cv2.line(self.debug_img_bg,
                         pt1=(vline, 20),
                         pt2=(vline, 300),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "-20cm", (120-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "  0cm", (200-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "+20cm", (280-25, 300+15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw horizontal lines of the grid
            for hline in np.arange(100, 301, 40):
                cv2.line(self.debug_img_bg,
                         pt1=(40, hline),
                         pt2=(360, hline),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "20cm", (2, 220+3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, " 0cm", (2, 300+3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw robot marker at the center
            cv2.line(self.debug_img_bg,
                     pt1=(200 + 0, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 + 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 - 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

        # map segment color variables to BGR colors
        color_map = {Segment.WHITE: (255, 255, 255),
                     Segment.RED: (0, 0, 255),
                     Segment.YELLOW: (0, 255, 255)}

        image = self.debug_img_bg.copy()

        # plot every segment if both ends are in the scope of the image (within 50cm from the origin)
        for segment in seg_list.segments:
            if not np.any(np.abs([segment.points[0].x, segment.points[0].y,
                                  segment.points[1].x, segment.points[1].y]) > 0.50):
                cv2.line(image,
                         pt1=(int(segment.points[0].y * -400) + 200, int(segment.points[0].x * -400) + 300),
                         pt2=(int(segment.points[1].y * -400) + 200, int(segment.points[1].x * -400) + 300),
                         color=color_map.get(segment.color, (0, 0, 0)),
                         thickness=1)

        return image


if __name__ == '__main__':
    ground_projection_node = GroundProjectionNode(node_name='ground_projection')
    rospy.spin()

