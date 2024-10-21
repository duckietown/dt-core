#!/usr/bin/env python3
import json
from typing import Optional

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Segment, SegmentList, AntiInstagramThresholds
from line_detector import LineDetector, ColorRange, plotSegments, plotMaps
from image_processing.anti_instagram import AntiInstagram

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam

from dt_computer_vision.camera import CameraModel, Pixel, NormalizedImagePoint


class LineDetectorNode(DTROS):
    """
    The ``LineDetectorNode`` is responsible for detecting the line white, yellow and red line segment in an
    image and is used for lane localization.

    Upon receiving an image, this node reduces its resolution, cuts off the top part so that only the
    road-containing part of the image is left, extracts the white, red, and yellow segments and publishes
    them.
    The main functionality of this node is implemented in the :py:class:`line_detector.LineDetector` class.

    The performance of this node can be very sensitive to its configuration parameters. Therefore, it also
    provides a number of debug topics which can be used for fine-tuning these parameters. These configuration
    parameters can be changed dynamically while the node is running via ``rosparam set`` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~line_detector_parameters (:obj:`dict`): A dictionary with the parameters for the detector.
                The full list can be found in :py:class:`line_detector.LineDetector`.
        ~colors (:obj:`dict`): A dictionary of colors and color ranges to be detected in the image.
                The keys (color names) should match the ones in the Segment message definition, otherwise an
                exception will be thrown! See the ``config`` directory in the node code for the default
                ranges.
        ~scale (:obj:`float`): A scaling factor to apply to the image before running the line detector.
                Lower resolution would result in faster detection but lower performance.
        ~top_cutoff (:obj:`int`): The number of rows to be removed from the top of the image
                _after_ scaling.

    Subscriber:
        ~camera_node/image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): The camera images
        ~anti_instagram_node/thresholds(:obj:`duckietown_msgs.msg.AntiInstagramThresholds`): The thresholds
            to do color correction

    Publishers:
        ~segment_list (:obj:`duckietown_msgs.msg.SegmentList`): A list of the detected segments.
            Each segment is an :obj:`duckietown_msgs.msg.Segment` message
        ~debug/segments/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the segments
            drawn on the input image
        ~debug/edges/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the Canny edges
            drawn on the input image
        ~debug/maps/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the regions falling
            in each color range drawn on the input image
        ~debug/ranges_HS (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the
            input image and the color ranges, Hue-Saturation projection
        ~debug/ranges_SV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the
            input image and the color ranges, Saturation-Value projection
        ~debug/ranges_HV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the
            input image and the color ranges, Hue-Value projection

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
            fsm_controlled=True
        )

        # Define parameters
        self._line_detector_parameters = rospy.get_param("~line_detector_parameters")
        self._veh = rospy.get_param("~veh")
        self._scale = rospy.get_param("~scale")
        self._top_cutoff = rospy.get_param("~top_cutoff")
        self._colors = DTParam("~colors")

        # TODO: use TurboJPEG instead
        self.bridge = CvBridge()

        # The thresholds to be used for AntiInstagram color correction
        self.ai_thresholds_received = False
        self.anti_instagram_thresholds = dict()
        self.ai = AntiInstagram()

        # This holds the colormaps for the debug/ranges images after they are computed once
        self.colormaps = dict()

        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self._line_detector_parameters)

        # Update the color ranges objects
        self.color_ranges = {}
        self.on_colors_range_change()
        self._colors.register_update_callback(self.on_colors_range_change)

        self.camera: Optional[CameraModel] = None

        # Publishers
        self.pub_lines = rospy.Publisher(
            "~segment_list", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_d_segments = rospy.Publisher(
            "~debug/segments/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_edges = rospy.Publisher(
            "~debug/edges/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_maps = rospy.Publisher(
            "~debug/maps/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        # these are not compressed because compression adds undesired blur
        self.pub_d_ranges_HS = rospy.Publisher(
            "~debug/ranges_HS/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_SV = rospy.Publisher(
            "~debug/ranges_SV/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_HV = rospy.Publisher(
            "~debug/ranges_HV/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        # Subscribers
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_image = rospy.Subscriber(
            "~image/compressed", CompressedImage, self.image_cb, buff_size=10000000, queue_size=1
        )

        self.sub_thresholds = rospy.Subscriber(
            "~thresholds", AntiInstagramThresholds, self.thresholds_cb, queue_size=1
        )

        # Check if CUDA is available
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.loginfo("Using CUDA GPU for line detection.")
            self.cuda_enabled = True
        else:
            self.loginfo("Using the CPU for line detection.")
            self.cuda_enabled = False

    def on_colors_range_change(self):
        self.color_ranges = {
            color: ColorRange.fromDict(d)
            for color, d in list(self._colors.value.items())
        }
        self.loginfo(f"Color range changed to {json.dumps(self._colors.value)}")

    def thresholds_cb(self, thresh_msg):
        # TODO: these should be DTParam
        self.anti_instagram_thresholds["lower"] = thresh_msg.low
        self.anti_instagram_thresholds["higher"] = thresh_msg.high
        self.ai_thresholds_received = True
        # TODO: unsubscribe here

    def cb_camera_info(self, msg: CameraInfo):
        """
        Initializes a :py:class:`image_processing.GroundProjectionGeometry` object and a
        :py:class:`image_processing.Rectify` object for image rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.

        """
        if self.camera is None:
            self.camera = CameraModel(
                width=msg.width,
                height=msg.height,
                K=np.array(msg.K).reshape((3, 3)),
                D=np.array(msg.D),
                P=np.array(msg.P).reshape((3, 4)),
            ).scaled(self._scale).cropped(top=self._top_cutoff)
            # unsubscribe from camera info topic
            self.loginfo("Camera parameters received, unsubscribing.")
            self.sub_camera_info.switch_off()

    def image_cb(self, image_msg):
        """
        Processes the incoming image messages.

        Performs the following steps for each incoming image:

        #. Performs color correction
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows of pixels
        #. Extracts the line segments in the image using :py:class:`line_detector.LineDetector`
        #. Converts the coordinates of detected segments to normalized ones
        #. Creates and publishes the resultant :obj:`duckietown_msgs.msg.SegmentList` message
        #. Creates and publishes debug images if there is a subscriber to the respective topics

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """
        if self.camera is None:
            return

        # Decode from compressed image with OpenCV
        try:
            obtained_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr(f"Could not decode image: {e}")
            return
        
        # Perform color correction
        if self.ai_thresholds_received:
            obtained_image = self.ai.apply_color_balance(
                self.anti_instagram_thresholds["lower"], self.anti_instagram_thresholds["higher"], obtained_image
            )

        if self.cuda_enabled:
            gpu_image = cv2.cuda_GpuMat()
            gpu_image.upload(obtained_image)
        else:
            gpu_image = obtained_image

        # Resize the gpu_image to the desired dimensions
        height_original, width_original = gpu_image.shape[0:2]
        img_size = (self._img_size[1], self._img_size[0])
        if img_size[0] != width_original or img_size[1] != height_original:
            if self.cuda_enabled:
                gpu_image = cv2.cuda.resize(gpu_image, img_size, interpolation=cv2.INTER_NEAREST)
            else:
                gpu_image = cv2.resize(gpu_image, img_size, interpolation=cv2.INTER_NEAREST)

        gpu_image = gpu_image[self._top_cutoff :, :, :]

        # mirror the gpu_image if left-hand traffic mode is set
        if self._traffic_mode.value == "LHT":
            gpu_image = np.fliplr(gpu_image)

        # Extract the line segments for every color
        self.detector.setImage(gpu_image)
        detections = {
            color: self.detector.detectLines(ranges) for color, ranges in list(self.color_ranges.items())
        }

        # Construct a SegmentList
        segment_list = SegmentList()
        segment_list.header.stamp = image_msg.header.stamp

        # # Fill in the segment_list with all the detected segments
        for color, det in list(detections.items()):
            # Get the ID for the color from the Segment msg definition, throw and exception otherwise
            if len(det.lines) > 0 and len(det.normals) > 0:
                try:
                    color_id = getattr(Segment, color)
                except AttributeError:
                    self.logerr(f"Color name {color} is not defined in the Segment message")
                    continue
                # itearate over segments
                for x0, y0, x1, y1, norm_x, norm_y in np.hstack((det.lines, det.normals)):
                    segment = Segment()
                    segment.color = color_id
                    # normalized point0
                    p0_px: Pixel = Pixel(x0, y0)
                    p0_nc: NormalizedImagePoint = self.camera.pixel2vector(p0_px)
                    # normalized point1
                    p1_px: Pixel = Pixel(x1, y1)
                    p1_nc: NormalizedImagePoint = self.camera.pixel2vector(p1_px)
                    # populate segments
                    segment.pixels_normalized[0].x = p0_nc.x
                    segment.pixels_normalized[0].y = p0_nc.y
                    segment.pixels_normalized[1].x = p1_nc.x
                    segment.pixels_normalized[1].y = p1_nc.y
                    segment.normal.x = norm_x
                    segment.normal.y = norm_y
                    # populate list
                    segment_list.segments.append(segment)

        # Publish the message
        self.pub_lines.publish(segment_list)
        
        if self.cuda_enabled:
            # Download the image from gpu memory
            image = gpu_image.download()
        else:
            # Just rename appropriately the image variable
            image = gpu_image

        # If there are any subscribers to the debug topics, generate a debug image and publish it
        if self.pub_d_segments.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in list(detections.items())}
            debug_img = plotSegments(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_segments.publish(debug_image_msg)

        if self.pub_d_edges.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.detector.canny_edges)
            debug_image_msg.header = image_msg.header
            self.pub_d_edges.publish(debug_image_msg)

        if self.pub_d_maps.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in list(detections.items())}
            debug_img = plotMaps(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_maps.publish(debug_image_msg)

        for channels in ["HS", "SV", "HV"]:
            publisher = getattr(self, f"pub_d_ranges_{channels}")
            if publisher.get_num_connections() > 0:
                debug_img = self._plot_ranges_histogram(channels)
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
                debug_image_msg.header = image_msg.header
                publisher.publish(debug_image_msg)

    def _plot_ranges_histogram(self, channels):
        """Utility method for plotting color histograms and color ranges.

        Args:
            channels (:obj:`str`): The desired two channels, should be one of ``['HS','SV','HV']``

        Returns:
            :obj:`numpy array`: The resultant plot image

        """
        channel_to_axis = {"H": 0, "S": 1, "V": 2}
        axis_to_range = {0: 180, 1: 256, 2: 256}

        # Get which is the third channel that will not be shown in this plot
        missing_channel = "HSV".replace(channels[0], "").replace(channels[1], "")

        hsv_im = self.detector.hsv
        # Get the pixels as a list (flatten the horizontal and vertical dimensions)
        hsv_im = hsv_im.reshape((-1, 3))

        channel_idx = [channel_to_axis[channels[0]], channel_to_axis[channels[1]]]

        # Get only the relevant channels
        x_bins = np.arange(0, axis_to_range[channel_idx[1]] + 1, 2)
        y_bins = np.arange(0, axis_to_range[channel_idx[0]] + 1, 2)
        h, _, _ = np.histogram2d(
            x=hsv_im[:, channel_idx[0]], y=hsv_im[:, channel_idx[1]], bins=[y_bins, x_bins]
        )
        # Log-normalized histogram
        np.log(h, out=h, where=(h != 0))
        h = (255 * h / np.max(h)).astype(np.uint8)

        # Make a color map, for the missing channel, just take the middle of the range
        if channels not in self.colormaps:
            colormap_1, colormap_0 = np.meshgrid(x_bins[:-1], y_bins[:-1])
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]] / 2)

            channel_to_map = {channels[0]: colormap_0, channels[1]: colormap_1, missing_channel: colormap_2}

            self.colormaps[channels] = np.stack(
                [channel_to_map["H"], channel_to_map["S"], channel_to_map["V"]], axis=-1
            ).astype(np.uint8)

            if self.cuda_enabled:
                self.colormaps[channels] = cv2.cuda.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)
            else:
                self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        if self.cuda_enabled:
            im = cv2.cuda.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        else:
            im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
            
        im = cv2.addWeighted(im, 0.5, self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in list(self.color_ranges.items()):
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            if self.cuda_enabled:
                color = cv2.cuda.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int).tolist()
            else:
                color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int).tolist()

            for i in range(len(color_range.low)):
                cv2.rectangle(
                    im,
                    pt1=(
                        (color_range.high[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.high[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    pt2=(
                        (color_range.low[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.low[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    color=color,
                    lineType=cv2.LINE_4,
                )
        # ---
        return im


if __name__ == "__main__":
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name="line_detector_node")
    # Keep it spinning to keep the node alive
    rospy.spin()
