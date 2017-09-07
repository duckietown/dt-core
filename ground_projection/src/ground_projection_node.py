#!/usr/bin/env python

import rospy
import cv2
from duckietown_msgs import (Pixel, Vector2D, Segment, SegmentList)
from ground_projection import (EstimateHomography, GetGroundCoord, GetImageCoord)
from sensors_msgs import (Image, CameraInfo, image_encodings)
from cv_bridge import CvBridge
import numpy as np
from ground_projection.GroundProjection import *


class GroundProjectionNode(object):

    def __init__(self):
        self.node_name="Ground Projection"
        self.gp = GroundProjection()
        self.active = True
        self.bridge=CvBridge()

        # Params 

        gp.robot_name = rospy.get_param("robot_name","shamrock")
        gp.rectified_input_ = rospy.get_param("rectified_input", False)
        self.image_channel_name = "image_raw"
        
        # Subs and Pubs
        self.pub_lineseglist_ = rospy.Publisher("lineseglist_out",SegmentList, queue_size=1)
        self.sub_lineseglist) = rospy.Subscriber("lineseglist_in",SegmentList, self.lineseglist_cb)
        
        
        # TODO prepare services
        self.service_homog_ = rospy.Service("estimate_homography", EstimateHomography, self.estimate_homography_cb)
        self.service_gnd_coord_ = rospy.Service("get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cv)
        self.service_img_coord_ = rospy.Service("get_image_coordinate", GetImageCoord, self.get_image_coordinate_cv)

        
    def rectifyImage(self,img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="mono8")
        except CvBridgeError as e:
            logger.error(e)
        return gp.rectify(cv_image)

    def lineseglist_cb(self,seglist_msg):
        seglist_out = SegmentList()
        for received_segment in seglist_msg:
            new_segment = Segment()
            new_segment.points[0] = gp.pixel2ground(received_segment.pixels_normalized[0])
            new_segment.points[1] = gp.pixel2ground(received_segment.pixels_normalized[1])
            seglist_out.segments.push(new_segment)
        pub_lineseglist.publish(seglist_out)

    def get_ground_coordinate_cb(req):
        return GetGroundCoordResponse(gp.pixel2ground(req.normalized_uv))

    def get_image_coordinate_cb(req):
        return GetImageCoordResponse(gp.ground2pixel(req.gp))

    def estimate_homography_cb(req):
        board_w = rospy.get_param("board_w",7)
        board_h = rospy.get_param("board_h",5)
        square_size = rospy.get_param("square_size",0.031)
        board_size = cv2.Size(board_w,board_h)
        x_offset = rospy.get_param("x_offset",0.191)
        y_offset = rospy.get_param("y_offset",-0.93)
        offset = cv2.Point2f(x_offset,y_offset)
        
        logger.info("Waiting for raw image on topic"+self.image_topic_name)
        img_msg = rospy.wait_for_message(self.image_topic_name,Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="mono8")
        except CvBridgeError as e:
            logger.error(e)
        gp.estimate_homography(cv_image, board_size, offset)
        
    def onShutdown(self):
        rospy.loginfo("[GroundProjectionNode] Shutdown.")
        
if __name__ == '__main__':
    rospy.init_node('ground_projection',anonymous=False)
    ground_projection_node = GroundProjectionNode()
    rospy.on_shutdown(ground_projection_node.onShutdown)
    rospy.spin()
