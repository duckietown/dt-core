#!/usr/bin/env python

from cv_bridge import CvBridge
from duckietown import DTROS
from duckietown_msgs.msg import Segment, SegmentList
from line_detector import LineDetector

class LineDetectorNode(DTROS):
    """Handles the imagery.

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~line_detector_parameters'] = None
        self.parameters['~colors'] = None
        self.updateParameters()

        self.bridge = CvBridge()

    def cbParametersChanged(self):

        # Create a LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self.parameters['~line_detector_parameters'])


if __name__ == '__main__':
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name='line_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()