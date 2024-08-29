#!/usr/bin/env python3

from typing import Optional

from dt_computer_vision.camera.homography import Homography
from dt_computer_vision.camera.types import CameraModel
from dt_computer_vision.ground_projection.ground_projector import GroundProjector

import numpy as np

from dt_computer_vision.optical_flow.optical_flow import OpticalFlow
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point as PointMsg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Range
from opencv_apps.msg import FlowArrayStamped, Flow


class OpticalFlowNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OpticalFlowNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION
        )

        # Initialize the parameters
        self.process_frequency = DTParam("~process_frequency", param_type=ParamType.INT)
        self.base_homography_height = DTParam(
            "~base_homography_height", param_type=ParamType.FLOAT
        )

        # TODO: make sure this parameter gets loaded at the right time
        # Retrieve the resize scale from the ROS parameter service
        self.resize_scale = rospy.get_param("/resized/scale_width")

        # obj attrs
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()

        self.camera: Optional[CameraModel] = None
        self._camera_info_initialized = False
        self.homography: Optional[Homography] = None
        self.projector: Optional[GroundProjector] = None

        # TODO: this should be deprecated as we are obtaining the optical flow vector from another node
        # optical flow setup
        self.optical_flow = OpticalFlow(1, 1, self.resize_scale)
        self.motion_vectors: Optional[np.ndarray] = None
        self.locations_px: Optional[np.ndarray] = None

        self._range: float = 0.0
        self._scale: float = self._range / self.base_homography_height.value

        # Publishers
        self.pub_debug_image = rospy.Publisher(
            "~debug/image/compressed", CompressedImage, queue_size=1
        )
        self.pub_odometry = rospy.Publisher("~visual_odometry", Odometry, queue_size=1)
        self.pub_motion_vectors = rospy.Publisher(
            "~lineseglist_out", SegmentList, queue_size=1
        )

        # Subscriber
        self.sub_motion_vectors = rospy.Subscriber(
            "~motion_vectors", FlowArrayStamped, self.cb_motion_vectors, queue_size=1
        )
        self.sub_range = rospy.Subscriber(
            "~range", Range, self.cb_new_range, queue_size=1
        )
        self.sub_projected_motion_vectors = rospy.Subscriber(
            "~projected_motion_vectors",
            SegmentList,
            self.cb_projected_motion_vectors,
            queue_size=1,
        )

        self.loginfo("Initialization completed.")

    def cb_motion_vectors(self, msg: FlowArrayStamped):
        """
        Callback for the motion vectors from the optical flow algorithm.
        Repack them as SegmentList and publish them to the ground projector.
        """
        segment_list = SegmentList()
        segment_list.header = msg.header

        if msg.flow is None:
            rospy.logwarn("No motion vectors received.")
            return

        segment_list.segments = []

        for flow in msg.flow:
            flow: Flow
            segment = Segment(
                points=[
                    PointMsg(x=flow.point.x, y=flow.point.y),
                    PointMsg(
                        x=flow.point.x + flow.velocity.x * 1,
                        y=flow.point.y + flow.velocity.y * 1,
                    ),
                ]
            )
            segment_list.segments.append(segment)

        self.pub_motion_vectors.publish(segment_list)

    def cb_projected_motion_vectors(self, msg: SegmentList):
        """
        Callback for the motion vectors from the optical flow algorithm.
        It grabs the projected motion vectors and computes the velocity vector using a mean.

        Args:
            msg: the SegmentList message containing the projected motion vectors

        """

        now = rospy.Time.now()
        # if now - self.last_stamp < rospy.Duration.from_sec(1.0 / self.process_frequency.value):
        #     return

        # TODO: We need to scale these depending on the parameter set in the image resize node

        if msg.segments is None:
            rospy.logwarn_throttle(period=1, msg="Empty motion vectors array received.")
            return

        if self.motion_vectors is None:
            num_motion_vectors = len(msg.segments)
            self.motion_vectors = np.zeros((2, num_motion_vectors))
            self.locations_px = np.zeros((2, num_motion_vectors))

        for i, flow in enumerate(msg.segments):
            flow: Segment
            self.motion_vectors[:, i] = np.array(
                [
                    flow.points[1].x - flow.points[0].x,
                    flow.points[1].y - flow.points[0].y,
                ]
            )

        if self.pub_debug_image.get_num_connections() > 0:
            # TODO: implement visualization
            return
            vis = self.optical_flow.create_debug_visualization(
                msg, self.locations_px, debug_str, self.motion_vectors
            )
            self.pub_debug_image.publish(self.bridge.cv2_to_compressed_imgmsg(vis))

        velocity = np.mean(np.array(self.motion_vectors), axis=0)

        # velocity = self.optical_flow.compute_velocity_vector(self.motion_vectors)

        # Rotate the velocity vector 90 degrees clockwise to match the odometry frame
        velocity = np.array([velocity[1], velocity[0]])

        assert velocity.shape == (2,), f"Velocity: {velocity}"
        self.logdebug(f"Computed velocity vector [m/s]: {velocity}")

        # Publish the optical flow vector as odometry
        odometry_msg = Odometry()
        odometry_msg.header.stamp = now

        # TODO: change this to the correct frame
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.twist.twist.linear.x = velocity[0]
        odometry_msg.twist.twist.linear.y = velocity[1]

        self.pub_odometry.publish(odometry_msg)

        # self.last_stamp = now

    def cb_new_range(self, msg: Range):
        self._range = msg.range
        self._scale = self._range / self.base_homography_height.value


if __name__ == "__main__":
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()
