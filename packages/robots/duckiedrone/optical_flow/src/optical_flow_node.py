#!/usr/bin/env python3

import os
from typing import Union

import cv2
from dt_computer_vision.camera.homography import HomographyToolkit, Homography
from dt_computer_vision.camera.types import CameraModel
from dt_computer_vision.ground_projection.ground_projector import GroundProjector
import numpy as np

from dt_computer_vision.optical_flow.optical_flow import OpticalFlow
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Range, CameraInfo


class OpticalFlowNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OpticalFlowNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Initialize the parameters
        self.process_frequency = DTParam("~process_frequency", param_type=ParamType.INT)
        self.track_len = DTParam("~track_len", param_type=ParamType.INT)
        self.detect_interval = DTParam("~detect_interval", param_type=ParamType.INT)
        self.base_homography_pixel_per_meter = DTParam("~base_homography_pixel_per_meter", param_type=ParamType.INT)
        self.virtual_camera_height = DTParam("~virtual_camera_height", param_type=ParamType.FLOAT)

        self.resize_scale = DTParam(
            "~img_scale",
            default=1.0,
            help="Scale the input image to the optical flow algorithm by this factor.",
            param_type=ParamType.FLOAT,
            max_value=1.0,
            min_value=0.0
        )
        
        self.scaled_height : float = 1.0
        self.scaled_width : float = 1.0

        # obj attrs
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()
        
        self.camera : CameraModel = None
        self._camera_info_initialized = False
        self.homography: Homography = None
        self.projector: GroundProjector = None

        # optical flow setup
        self.optical_flow = OpticalFlow(
            self.track_len.value, self.detect_interval.value, self.resize_scale.value
        )
        self._range : float = 0.0

        # TODO: these should be determined dynamically
        self.h_cropped, self.w_cropped = 240, 320

        # Publishers
        self.pub_debug_image = rospy.Publisher("~debug/image/compressed", CompressedImage, queue_size=1)
        self.pub_debug_projected_image = rospy.Publisher("~debug/image/projected/compressed", CompressedImage, queue_size=1)
        self.pub_odometry = rospy.Publisher("~visual_odometry", Odometry, queue_size=1)

        # Subscriber
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cb_image, queue_size=1)
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_height = rospy.Subscriber("~range", Range, self.cb_new_range, queue_size=1)

        self.loginfo("Initialization completed.")

    def cb_camera_info(self, msg: CameraInfo):
        if not self._camera_info_initialized:
            self.camera = CameraModel(
                height=msg.height,
                width=msg.width,
                K=np.array(msg.K).reshape((3, 3)),
                D=np.array(msg.D),
                R=np.array(msg.R).reshape((3, 3)),
                P=np.array(msg.P).reshape((3, 4)),
            )
            
            # TODO: remove this hardcoded homography, fix the homography loading
            self.homography = Homography([[7.68649577e-02, 4.80407934e-02, 4.79735399e-02],
                                [2.35921238e-04, 1.98051018e-01, 1.04001771e-01],
                                [3.19667170e-02, 9.15595543e-01, 1.00000000e+00]])

            from dt_computer_vision.camera.homography import interpolate_homography

            R2 = np.eye(3)
            tvec2 = np.array([0, 0.0, self.virtual_camera_height.value]).reshape(3, 1)

            self.camera.H =  interpolate_homography(self.homography, tvec2, R2, self.camera)

            self.loginfo(f"Camera model initialized: {self.camera}")
            self.projector = GroundProjector(self.camera)


        self._camera_info_initialized = True

    def cb_image(self, image_msg : CompressedImage):
        if not self._camera_info_initialized:
            rospy.logdebug("Cannot process image, camera info not initialized.")
            return

        now = rospy.Time.now()
        if now - self.last_stamp < rospy.Duration.from_sec(1.0 / self.process_frequency.value):
            return

        t_now = now.to_sec()
        t_stamp = image_msg.header.stamp.to_sec()
        if abs(t_now - t_stamp) > 0.3:
            return

        image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        delta_t = float(t_now - self.last_stamp.to_sec())

        image = self.camera.rectifier.rectify(image)
        
        # Compute the optical flow
        displacements_array, motion_vectors, locations_px, debug_str = (
            self.optical_flow.compute_motion_vectors(image, delta_t)
        )
        
        if self.pub_debug_image.get_num_connections() > 0:
            vis = self.optical_flow.create_debug_visualization(
                image, locations_px, debug_str, motion_vectors
            )
            self.pub_debug_image.publish(self.bridge.cv2_to_compressed_imgmsg(vis))

        projected_motion_vectors, projected_locations = self.optical_flow.project_motion_vectors(
            motion_vectors, locations_px, self.camera, self.camera.H
        )
        
        if self.pub_debug_projected_image.get_num_connections() > 0:
            projected_image = cv2.warpPerspective(image, self.camera.H, (self.camera.width, self.camera.height))
            
            vis = self.optical_flow.create_debug_visualization(
                        projected_image,
                        projected_locations,
                        debug_str,
                        motion_vectors=projected_motion_vectors,
                    )
            self.pub_debug_projected_image.publish(self.bridge.cv2_to_compressed_imgmsg(vis))

        if debug_str:
            rospy.logdebug(debug_str)

        # TODO: publish the motion vectors, useful for debugging
        

        velocity = self.optical_flow.compute_velocity_vector(projected_motion_vectors)
        
        # TODO: is this the correct way to handle the velocity vector?
        if velocity.shape == (2,1):
            velocity = velocity.squeeze()
        else:
            velocity = np.array([0, 0])
        
        # Remove one dimension in the array
        assert velocity.shape == (2,) , f"Velocity: {velocity}"
        self.logdebug(f"Computed velocity vector [px/s]: {velocity}")

        # Publish the optical flow vector as odometry
        odometry_msg = Odometry()
        odometry_msg.header.stamp = now

        # TODO: change this to the correct frame
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.twist.twist.linear.x = velocity[0] / self.base_homography_pixel_per_meter.value
        odometry_msg.twist.twist.linear.y = velocity[1] / self.base_homography_pixel_per_meter.value 

        self.pub_odometry.publish(odometry_msg)

        self.last_stamp = now

    def cb_new_range(self, msg : Range):
        return
        if self._camera_info_initialized:
            self._range = msg.range
            self._update_projector_homography(self._range)
            self._homography_to_ground_initialized = True
        else:
            rospy.logdebug("Received new range but cannot update projector homography, camera info not initialized.")
        
    def _update_projector_homography(self, range: float):
        return
        # Modify the camera's homography to account for the height of the drone
        H_floor = np.linalg.inv(
            range
            * np.array([[0, 0, self.camera.cx], [0, 0, self.camera.cy], [0, 0, 1]])
        )

        self.camera.H = H_floor
        
    ##########################
    def load_homography(self) -> Union[Homography, None]:
        """
        Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`Homography`: the loaded homography matrix

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
            H : Homography = HomographyToolkit.load_from_disk(cali_file, return_date=False) # type: ignore
            return H.reshape((3, 3))
        except Exception as e:
            msg = f"Error in parsing calibration file {cali_file}:\n{e}"
            self.logerr(msg)
            rospy.signal_shutdown(msg)


if __name__ == "__main__":
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()
