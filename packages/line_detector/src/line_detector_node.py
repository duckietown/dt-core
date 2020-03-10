#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList
from line_detector import LineDetector, ColorRange, plotSegments, plotMaps


class LineDetectorNode(DTROS):
    """Handles the imagery.

        color names should match the ones in the Segment message, otherwise an exception will be thrown
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~line_detector_parameters'] = None
        self.parameters['~colors'] = None
        self.parameters['~img_size'] = None
        self.parameters['~top_cutoff'] = None
        self.updateParameters()

        self.bridge = CvBridge()
        self.colormaps = dict()  #: holds the colormaps for the debug/ranges images after they are computed once

        # Subscribers
        self.sub_image = self.subscriber("~corrected_image/compressed", CompressedImage, self.cbImage,
                                         queue_size=1)

        # Publishers
        self.pub_lines = self.publisher("~segment_list", SegmentList, queue_size=1)
        self.pub_d_segments = self.publisher("~debug/segments/compressed", CompressedImage, queue_size=1)
        self.pub_d_edges = self.publisher("~debug/edges/compressed", CompressedImage, queue_size=1)
        self.pub_d_maps = self.publisher("~debug/maps/compressed", CompressedImage, queue_size=1)
        # these are not compressed, because compression adds undesired blur
        self.pub_d_ranges_HS = self.publisher("~debug/ranges_HS", Image, queue_size=1)
        self.pub_d_ranges_SV = self.publisher("~debug/ranges_SV", Image, queue_size=1)
        self.pub_d_ranges_HV = self.publisher("~debug/ranges_HV", Image, queue_size=1)

    def cbParametersChanged(self):
        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self.parameters['~line_detector_parameters'])
        # Update the color ranges objects
        self.color_ranges = {color: ColorRange.fromDict(d) for color, d in self.parameters['~colors'].iteritems()}

    def cbImage(self, image_msg):

        # Decode from compressed image with OpenCV
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        # Resize the image to the desired dimensions
        height_original, width_original = image.shape[0:2]
        if self.parameters['~img_size'][0] != height_original or self.parameters['~img_size'][1] != width_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image = cv2.resize(image, (self.parameters['~img_size'][1], self.parameters['~img_size'][0]),
                               interpolation=cv2.INTER_NEAREST)
        image = image[self.parameters['~top_cutoff']:, :, :]

        # Extract the line segments for every color
        self.detector.setImage(image)
        detections = {color: self.detector.detectLines(ranges) for color, ranges in self.color_ranges.iteritems()}

        # Construct a SegmentList
        segment_list = SegmentList()
        segment_list.header.stamp = image_msg.header.stamp

        # Remove the offset in coordinates coming from the removing of the top part and
        arr_cutoff = np.array([0, self.parameters['~top_cutoff'], 0, self.parameters['~top_cutoff']])
        arr_ratio = np.array([1. / self.parameters['~img_size'][1], 1. / self.parameters['~img_size'][0],
                              1. / self.parameters['~img_size'][1], 1. / self.parameters['~img_size'][0]])

        # Fill in the segment_list with all the detected segments
        for color, det in detections.iteritems():
            # Get the ID for the color from the Segment msg definition
            # Throw and exception otherwise
            if len(det.lines) > 0 and len(det.normals) > 0:
                try:
                    color_id = getattr(Segment, color)
                    lines_normalized = (det.lines + arr_cutoff) * arr_ratio
                    segment_list.segments.extend(self.toSegmentMsg(lines_normalized, det.normals, color_id))
                except AttributeError:
                    self.logerr("Color name %s is not defined in the Segment message" % color)

        # Publish the message
        self.pub_lines.publish(segment_list)

        # If there are any subscribers to the debug topics, generate a debug image and publish it
        if self.pub_d_segments.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in detections.iteritems()}
            debug_img = plotSegments(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_segments.publish(debug_image_msg)

        if self.pub_d_edges.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.detector.canny_edges)
            debug_image_msg.header = image_msg.header
            self.pub_d_edges.publish(debug_image_msg)

        if self.pub_d_maps.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in detections.iteritems()}
            debug_img = plotMaps(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_maps.publish(debug_image_msg)

        for channels in ['HS', 'SV', 'HV']:
            publisher = getattr(self, 'pub_d_ranges_%s' % channels)
            if publisher.get_num_connections() > 0:
                debug_img = self.plotRangesHistogram(channels)
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                debug_image_msg.header = image_msg.header
                publisher.publish(debug_image_msg)

    @staticmethod
    def toSegmentMsg(lines, normals, color):
        segment_msg_list = []
        for x1, y1, x2, y2, norm_x, norm_y in np.hstack((lines, normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segment_msg_list.append(segment)
        return segment_msg_list

    def plotRangesHistogram(self, channels):

        channel_to_axis = {'H': 0, 'S': 1, 'V': 2}
        axis_to_range = {0: 180, 1: 256, 2: 256}

        # Get which is the third channel that will not be shown in this plot
        missing_channel = 'HSV'.replace(channels[0], '').replace(channels[1], '')

        hsv_im = self.detector.hsv
        # Get the pixels as a list (flatten the horizontal and vertical dimensions)
        hsv_im = hsv_im.reshape((-1, 3))

        channel_idx = [channel_to_axis[channels[0]], channel_to_axis[channels[1]]]

        # Get only the relevant channels
        x_bins = np.arange(0, axis_to_range[channel_idx[1]] + 1, 2)
        y_bins = np.arange(0, axis_to_range[channel_idx[0]] + 1, 2)
        h, _, _ = np.histogram2d(x=hsv_im[:, channel_idx[0]], y=hsv_im[:, channel_idx[1]],
                                 bins=[y_bins, x_bins])
        # Log-normalized histogram
        np.log(h, out=h, where=(h != 0))
        h = (255 * h / np.max(h)).astype(np.uint8)

        # Make a color map, for the missing channel, just take the middle of the range
        if channels not in self.colormaps:
            colormap_1, colormap_0 = np.meshgrid(x_bins[:-1], y_bins[:-1])
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]]/2)

            channel_to_map = {channels[0]: colormap_0,
                              channels[1]: colormap_1,
                              missing_channel: colormap_2}

            self.colormaps[channels] = np.stack([channel_to_map['H'], channel_to_map['S'], channel_to_map['V']], axis=-1).astype(np.uint8)
            self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        im = cv2.addWeighted(im, 0.5 , self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in self.color_ranges.iteritems():
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int)
            for i in range(len(color_range.low)):
                cv2.rectangle(im,
                              pt1=((color_range.high[i, channel_idx[1]]/2).astype(np.uint8), (color_range.high[i, channel_idx[0]]/2).astype(np.uint8)),
                              pt2=((color_range.low[i, channel_idx[1]]/2).astype(np.uint8), (color_range.low[i, channel_idx[0]]/2).astype(np.uint8)),
                              color=color,
                              lineType=cv2.LINE_4)


        return im


if __name__ == '__main__':
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name='line_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
