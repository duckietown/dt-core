#!/usr/bin/env python3

import os
from collections import defaultdict
from typing import Optional, Dict, List, Tuple

import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Segment as SegmentMsg, SegmentList, Vector2D
from geometry_msgs.msg import Point as PointMsg
from sensor_msgs.msg import CameraInfo, CompressedImage

from dt_class_utils import DTReminder
from dt_computer_vision.camera import CameraModel, Pixel, NormalizedImagePoint, BGRImage
from dt_computer_vision.camera.homography import ResolutionDependentHomography, HomographyToolkit, \
    ResolutionIndependentHomography
from dt_computer_vision.ground_projection import GroundProjector, GroundPoint
from dt_computer_vision.ground_projection.rendering import draw_grid_image, debug_image, Color

COLORS: Dict[int, Color] = {
    SegmentMsg.WHITE: (255, 255, 255),
    SegmentMsg.YELLOW: (0, 255, 255),
    SegmentMsg.RED: (0, 0, 255),
}
DEBUG_IMG_SIZE: Tuple[int, int] = (400, 400)


class GroundProjectionNode(DTROS):
    """
    This node projects the line segments detected in the image to the ground plane and in the robot's
    reference frame.
    In this way it enables lane localization in the 2D ground plane. This projection is performed using the
    homography
    matrix obtained from the extrinsic calibration procedure.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Subscribers:
        ~camera_info (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera. Needed for
        rectifying the segments.
        ~lineseglist_in (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from
        unrectified images

    Publishers:
        ~lineseglist_out (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane
        relative to the robot origin
        ~debug/ground_projection_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image
        that shows the robot relative to the projected segments. Useful to check if the extrinsic
        calibration is accurate.
    """

    bridge: CvBridge

    def __init__(self, node_name: str):
        # Initialize the DTROS parent class
        super(GroundProjectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION,
                                                   fsm_controlled=True)

        # TODO: use turboJPEG instead
        self.bridge = CvBridge()

        self.camera: Optional[CameraModel] = None
        self.ground_projector: Optional[GroundProjector] = None
        self.homography: Optional[ResolutionDependentHomography] = None
        self.debug_grid: BGRImage = draw_grid_image(DEBUG_IMG_SIZE)

        self._initialized: bool = False
        self._initialization_warn: DTReminder = DTReminder(period=5)
        self._first_processing_done: bool = False

        # subscribers
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber(
            "~lineseglist_in", SegmentList, self.lineseglist_cb, queue_size=1
        )

        # publishers
        self.pub_lineseglist = rospy.Publisher(
            "~lineseglist_out", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_debug_img = rospy.Publisher(
            "~debug/ground_projection_image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG,
        )

    def cb_camera_info(self, msg: CameraInfo):
        """
        Initializes a :py:class:`image_processing.GroundProjectionGeometry` object and a
        :py:class:`image_processing.Rectify` object for image rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.

        """
        if self.ground_projector is None:
            self.camera = CameraModel(
                width=msg.width,
                height=msg.height,
                K=np.array(msg.K).reshape((3, 3)),
                D=np.array(msg.D),
                P=np.array(msg.P).reshape((3, 4)),
            )
            self.camera.H = self.load_extrinsics(self.camera)
            self.ground_projector = GroundProjector(self.camera)
            # unsubscribe from camera info topic
            self.loginfo("Camera parameters received, unsubscribing.")
            self.sub_camera_info.switch_off()
            # ---
            self._initialized = True

    def lineseglist_cb(self, seglist_msg):
        """
        Projects a list of line segments on the ground reference frame point by point by
        calling :py:meth:`pixel_to_ground`. Then publishes the projected list of segments.

        Args:
            seglist_msg (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from
            unrectified images

        """
        if not self._initialized:
            if self._initialization_warn.is_time():
                self.log("Waiting for a CameraInfo message", "warn")
            return

        colored_segments: Dict[Color, List[Tuple[GroundPoint, GroundPoint]]] = defaultdict(list)
        seglist_out = SegmentList()
        seglist_out.header = seglist_msg.header
        for received_segment in seglist_msg.segments:
            new_segment = SegmentMsg()
            # project start and end of the segment
            ground_points: List[GroundPoint] = []
            for i in range(2):
                pt: Vector2D = received_segment.pixels_normalized[i]
                ndp: NormalizedImagePoint = NormalizedImagePoint(pt.x, pt.y)
                # normalized distorted coordinates -> distorted pixels
                px: Pixel = self.camera.vector2pixel(ndp)
                # distorted pixels -> rectified pixels
                pxr: Pixel = self.camera.rectifier.rectify_pixel(px)
                # rectified pixel -> normalized (rectified) coordinates
                nrp: NormalizedImagePoint = self.camera.pixel2vector(pxr)
                # project image point onto the ground plane
                gp: GroundPoint = self.ground_projector.vector2ground(nrp)
                # create new point
                new_segment.points[i] = PointMsg(
                    x=gp.x,
                    y=gp.y,
                    z=0,
                )
                # add ground point to segment
                ground_points.append(gp)
            # color segment
            new_segment.color = received_segment.color
            # TODO what about normal and points?
            # add segment to list of segments
            seglist_out.segments.append(new_segment)
            colored_segments[COLORS[received_segment.color]].append(tuple(ground_points))
        # publish list of segments
        self.pub_lineseglist.publish(seglist_out)

        if not self._first_processing_done:
            self.log("First projected segments published.")
            self._first_processing_done = True

        if self.pub_debug_img.get_num_connections() > 0:
            image_w_segs = debug_image(colored_segments, DEBUG_IMG_SIZE, background_image=self.debug_grid)
            image_w_segs_msg = self.bridge.cv2_to_compressed_imgmsg(image_w_segs)
            image_w_segs_msg.header = seglist_out.header
            self.pub_debug_img.publish(image_w_segs_msg)

    def load_extrinsics(self, camera: CameraModel) -> Optional[ResolutionDependentHomography]:
        """
        Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`numpy array`: the loaded homography matrix

        """
        # load intrinsic calibration
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log(
                f"Can't find calibration file: {cali_file}.\n Using default calibration instead.", "warn"
            )
            cali_file = os.path.join(cali_file_folder, "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = "Found no calibration file ... aborting"
            self.logerr(msg)
            rospy.signal_shutdown(msg)

        # load calibration
        Hindep: Optional[ResolutionIndependentHomography] = None
        try:
            Hindep = HomographyToolkit.load_from_disk(cali_file)
        except BaseException as e:
            self.logerr(str(e))
            rospy.signal_shutdown(str(e))
            exit(1)

        # compute resolution-dependent homography
        H: ResolutionDependentHomography = Hindep.camera_specific(camera)
        return H


if __name__ == "__main__":
    ground_projection_node = GroundProjectionNode(node_name="ground_projection_node")
    rospy.spin()
