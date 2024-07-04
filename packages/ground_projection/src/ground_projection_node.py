#!/usr/bin/env python3

from dataclasses import dataclass
import os
from typing import Optional

import cv2
from dt_computer_vision.camera.calibration.extrinsics.boards import CalibrationBoard8by6
from dt_computer_vision.camera.calibration.extrinsics.chessboard import compute_homography_maps
from dt_computer_vision.camera.types import RegionOfInterest, Size
import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point as PointMsg
from image_processing.ground_projection_geometry import GroundProjectionGeometry, Point
from image_processing.rectification import Rectify
from sensor_msgs.msg import CameraInfo, CompressedImage

from dt_computer_vision.camera import CameraModel, BGRImage
from dt_computer_vision.ground_projection import GroundProjector

from dt_computer_vision.camera.homography import HomographyToolkit, ResolutionIndependentHomography
from duckietown.dtros import DTROS, NodeType, TopicType

@dataclass
class ImageProjectorConfig:
    roi : RegionOfInterest
    ppm : int

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
        ~image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Compressed image from the camera. 
        Only needed if you want to visualize the debug image (i.e. the image with the homography applied).

    Publishers:
        ~lineseglist_out (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane
        relative to the robot origin
        ~debug/ground_projection_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image
        that shows the robot relative to the projected segments. Useful to check if the extrinsic
        calibration is accurate.
        ~debug/projected_image/rectified/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image
        that shows the rectified image. Useful to check if the rectification is accurate.
        ~debug/projected_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image that shows
        the projected image. Useful to check if the homography is accurate.
    """

    bridge: CvBridge
    ground_projector: Optional[GroundProjectionGeometry]
    rectifier: Optional[Rectify]

    def __init__(self, node_name: str):
        # Initialize the DTROS parent class
        super(GroundProjectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION,
                                                   fsm_controlled=True)

        self.bridge = CvBridge()
        self.ground_projector: Optional[GroundProjectionGeometry] = None
        self.rectifier: Optional[Rectify] = None
        self.camera: Optional[CameraModel] = None
        self.homography: ResolutionIndependentHomography = None
        self.first_processing_done = False
        self.camera_info_received = False

        self._image_projector_config: dict = rospy.get_param(
            "~image_projector_configuration", None, type=dict
        )

        self.image_projector = ImageProjectorConfig(**self._image_projector_config)

        # subscribers
        self.sub_camera_info = rospy.Subscriber(
            "~camera_info", CameraInfo, self.cb_camera_info, queue_size=1
        )
        self.sub_lineseglist_ = rospy.Subscriber(
            "~lineseglist_in", SegmentList, self.lineseglist_cb, queue_size=1
        )
        self.sub_image = rospy.Subscriber("~image/compressed", CompressedImage, self.cb_process_image, queue_size=1)

        # publishers
        self.pub_lineseglist = rospy.Publisher(
            "~lineseglist_out", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_debug_road_view_img = rospy.Publisher(
            "~debug/ground_projection_image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG,
        )
        
        self.pub_debug_rectified_img = rospy.Publisher(
            "~debug/projected_image/rectified/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG,
        )
        
        self.pub_debug_projected_img = rospy.Publisher(
            "~debug/projected_image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG,
        )


        self.bridge = CvBridge()

        self.debug_img_bg = None

        # Seems to be never used:
        # self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography,
        # self.estimate_homography_cb)
        # self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord,
        # self.get_ground_coordinate_cb)
        # self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord,
        # self.get_image_coordinate_cb)

    def cb_camera_info(self, msg: CameraInfo):
        """
        Initializes a :py:class:`image_processing.GroundProjectionGeometry` object and a
        :py:class:`image_processing.Rectify` object for image rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.

        """
        if not self.camera_info_received:
            self.log("Received camera info message")
            # create camera object
            self.camera = CameraModel(
                width=msg.width,
                height=msg.height,
                K=np.reshape(msg.K, (3, 3)),
                D=msg.D,
                P=np.reshape(msg.P, (3, 4)),
            )
            self.homography = self.load_extrinsics(self.camera)
            self.camera.H = self.homography
            self.rectifier = Rectify(msg)
            self.ground_projector = GroundProjectionGeometry(
                im_width=msg.width, im_height=msg.height, homography=self.homography
            )
            self.projector = GroundProjector(self.camera)
            
            self.loginfo("Computing virtual camera maps using the homography")
            
            roi = RegionOfInterest(
                origin=Point(self.roi.origin.x, self.roi.origin.y),
                size=Size(self.roi.size.x, self.roi.size.y),
            )

            self.mapx, self.mapy, self.mask, self.virtual_camera_shape = (
                compute_homography_maps(self.camera, self.camera.H, self.image_projector.ppm, roi)
            )
            self.loginfo("Virtual camera maps computed")

        self.camera_info_received = True

    def pixel_msg_to_ground_msg(self, point_msg) -> PointMsg:
        """
        Creates a :py:class:`ground_projection.Point` object from a normalized point message from an
        unrectified
        image. It converts it to pixel coordinates and rectifies it. Then projects it to the ground plane and
        converts it to a ROS Point message.

        Args:
            point_msg (:obj:`geometry_msgs.msg.Point`): Normalized point coordinates from an unrectified
            image.

        Returns:
            :obj:`geometry_msgs.msg.Point`: Point coordinates in the ground reference frame.

        """
        # normalized coordinates to pixel:
        norm_pt = Point.from_message(point_msg)
        pixel = self.ground_projector.vector2pixel(norm_pt)
        # rectify
        rect = self.rectifier.rectify_point(pixel)
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
        """
        Projects a list of line segments on the ground reference frame point by point by
        calling :py:meth:`pixel_msg_to_ground_msg`. Then publishes the projected list of segments.

        Args:
            seglist_msg (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from
            unrectified images

        """
        if self.camera_info_received:
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
                self.log("First projected segments published.")
                self.first_processing_done = True

            if self.pub_debug_road_view_img.get_num_connections() > 0:
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.debug_image(seglist_out))
                debug_image_msg.header = seglist_out.header
                self.pub_debug_road_view_img.publish(debug_image_msg)
        else:
            self.log("Waiting for a CameraInfo message", "warn")

    def load_extrinsics(self, camera: CameraModel) -> ResolutionIndependentHomography:
        """
        Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`numpy array`: the loaded homography matrix

        """
        # load extrinsic calibration
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log(
                f"Can't find calibration file: {cali_file}\n Using default calibration instead.", "warn"
            )
            cali_file = os.path.join(cali_file_folder, "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = "Found no calibration file ... aborting"
            self.logerr(msg)
            rospy.signal_shutdown(msg)

        try:
            Hindep: ResolutionIndependentHomography = HomographyToolkit.load_from_disk(cali_file)
            return Hindep.reshape((3, 3))
        except Exception as e:
            msg = f"Error in parsing calibration file {cali_file}:\n{e}"
            self.logerr(msg)
            rospy.signal_shutdown(msg)

    def debug_image(self, seg_list):
        """
        Generates a debug image with all the projected segments plotted with respect to the robot's origin.

        Args:
            seg_list (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane relative
            to the robot origin

        Returns:
            :obj:`numpy array`: an OpenCV image

        """
        # dimensions of the image are 1m x 1m so, 1px = 2.5mm
        # the origin is at x=200 and y=300

        # if that's the first call, generate the background
        if self.debug_img_bg is None:

            # initialize gray image
            self.debug_img_bg = np.ones((400, 400, 3), np.uint8) * 128

            # draw vertical lines of the grid
            for vline in np.arange(40, 361, 40):
                cv2.line(
                    self.debug_img_bg, pt1=(vline, 20), pt2=(vline, 300), color=(255, 255, 0), thickness=1
                )

            # draw the coordinates
            cv2.putText(
                self.debug_img_bg,
                "-20cm",
                (120 - 25, 300 + 15),
                cv2.FONT_HERSHEY_PLAIN,
                0.8,
                (255, 255, 0),
                1,
            )
            cv2.putText(
                self.debug_img_bg,
                "  0cm",
                (200 - 25, 300 + 15),
                cv2.FONT_HERSHEY_PLAIN,
                0.8,
                (255, 255, 0),
                1,
            )
            cv2.putText(
                self.debug_img_bg,
                "+20cm",
                (280 - 25, 300 + 15),
                cv2.FONT_HERSHEY_PLAIN,
                0.8,
                (255, 255, 0),
                1,
            )

            # draw horizontal lines of the grid
            for hline in np.arange(20, 301, 40):
                cv2.line(
                    self.debug_img_bg, pt1=(40, hline), pt2=(360, hline), color=(255, 255, 0), thickness=1
                )

            # draw the coordinates
            cv2.putText(
                self.debug_img_bg, "20cm", (2, 220 + 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1
            )
            cv2.putText(
                self.debug_img_bg, " 0cm", (2, 300 + 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1
            )

            # draw robot marker at the center
            cv2.line(
                self.debug_img_bg,
                pt1=(200 + 0, 300 - 20),
                pt2=(200 + 0, 300 + 0),
                color=(255, 0, 0),
                thickness=1,
            )

            cv2.line(
                self.debug_img_bg,
                pt1=(200 + 20, 300 - 20),
                pt2=(200 + 0, 300 + 0),
                color=(255, 0, 0),
                thickness=1,
            )

            cv2.line(
                self.debug_img_bg,
                pt1=(200 - 20, 300 - 20),
                pt2=(200 + 0, 300 + 0),
                color=(255, 0, 0),
                thickness=1,
            )

        # map segment color variables to BGR colors
        color_map = {Segment.WHITE: (255, 255, 255), Segment.RED: (0, 0, 255), Segment.YELLOW: (0, 255, 255)}

        image = self.debug_img_bg.copy()

        # plot every segment if both ends are in the scope of the image (within 50cm from the origin)
        for segment in seg_list.segments:
            if not np.any(
                np.abs([segment.points[0].x, segment.points[0].y, segment.points[1].x, segment.points[1].y])
                > 0.50
            ):
                cv2.line(
                    image,
                    pt1=(int(segment.points[0].y * -400) + 200, int(segment.points[0].x * -400) + 300),
                    pt2=(int(segment.points[1].y * -400) + 200, int(segment.points[1].x * -400) + 300),
                    color=color_map.get(segment.color, (0, 0, 0)),
                    thickness=1,
                )

        return image

    def cb_process_image(self, msg: CompressedImage):
        """
        Rectifies and projects the image to the ground plane and publishes it as a debug image.

        Args:
            msg (:obj:`sensor_msgs.msg.CompressedImage`): Compressed image from the camera

        """
        if self.camera_info_received:
            bgr_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rectified_image = self.camera.rectifier.rectify(bgr_image)

            projected_image = self.project_image(
                rectified_image, self.mapx, self.mapy, self.mask
            )

            if self.pub_debug_rectified_img.get_num_connections() > 0:
                self.pub_debug_rectified_img.publish(self.bridge.cv2_to_compressed_imgmsg(rectified_image))

            self.pub_debug_projected_img.publish(self.bridge.cv2_to_compressed_imgmsg(projected_image))
        else:
            self.log("Waiting for a CameraInfo message", "warn")

    @staticmethod
    def project_image(
        rectified_image: BGRImage, mapx: np.ndarray, mapy: np.ndarray, mask: np.ndarray
    ) -> BGRImage:
        """
        Projects a rectified image using the maps generated from the homography matrix.
        """
        img = cv2.remap(rectified_image, mapx, mapy, cv2.INTER_CUBIC)

        img = cv2.inpaint(img, mask, 3, cv2.INPAINT_NS)

        # flip and rotate the image so that it appears as it is seen from the camera
        img = cv2.flip(img, 0)
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


if __name__ == "__main__":
    ground_projection_node = GroundProjectionNode(node_name="ground_projection_node")
    rospy.spin()
