#!/usr/bin/env python

from cv_bridge import CvBridge
from duckietown import DTROS
from duckietown_msgs.msg import Segment, SegmentList
from line_detector import LineDetector, ColorRange


class LineDetectorNode(DTROS):
    """Handles the imagery.

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
        self.pub_image = self.publisher("~image_with_lines", Image, queue_size=1)

    def cbParametersChanged(self):
        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self.parameters['~line_detector_parameters'])
        # Update the color ranges objects
        self.color_ranges = {color: ColorRange.fromDict(d) for color, d in self.parameters['~colors'].iteritems()}

    def cbImage(self, image_msg):

        # Decode from compressed image with OpenCV
        try:
            image = bgr_from_jpg(image_msg.data)
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
        detections = {color: detectLines(ranges) for color, ranges in self.color_ranges.iteritems()}

        # Construct a SegmentList
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Remove the offset in coordinates coming from the removing of the top part and
        arr_cutoff = np.array([0, self.parameters['~top_cutoff'], 0, self.parameters['~top_cutoff']])
        arr_ratio = np.array([1./self.parameters['~img_size'][1], 1./self.parameters['~img_size'][0],
                              1./self.parameters['~img_size'][1], 1./self.parameters['~img_size'][0]])

if __name__ == '__main__':
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name='line_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
