#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Segment, SegmentList
from line_detector import LineDetector, ColorRange, plotDetections


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

        # Subscribers
        self.sub_image = self.subscriber("~corrected_image/compressed", CompressedImage, self.cbImage,
                                         queue_size=1)

        # Publishers
        self.pub_lines = self.publisher("~segment_list", SegmentList, queue_size=1)
        self.pub_image = self.publisher("~debug_image", CompressedImage, queue_size=1)

    def cbParametersChanged(self):
        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self.parameters['~line_detector_parameters'])
        # Update the color ranges objects
        self.color_ranges = {color: ColorRange.fromDict(d) for color, d in self.parameters['~colors'].iteritems()}

    def cbImage(self, image_msg):

        # Decode from compressed image with OpenCV
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        # Resize the image to the desired dimensions
        height_original, width_original = image.shape[0:2]
        if self.parameters['~img_size'][0] != height_original or self.parameters['~img_size'][1] != width_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image = cv2.resize(image, (self.parameters['~img_size'][1], self.parameters['~img_size'][0]),
                               interpolation=cv2.INTER_NEAREST)
        image = image[self.top_cutoff:, :, :]

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
                    det.lines = (det.lines + arr_cutoff) * arr_ratio
                    segment_list.segments.extend(self.toSegmentMsg(det.lines, det.normals, color_id))
                except AttributeError:
                    self.logerr("Color name %s is not defined in the Segment message" % color)

        # Publish the message
        self.pub_lines.publish(segment_list)

        # If there are any subscribers to the pub_image topic, generate a debug image and publish it
        if self.pub_image.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in detections.iteritem()}
            debug_img = plotDetections(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_img.publish(debug_image_msg)

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

if __name__ == '__main__':
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name='line_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
