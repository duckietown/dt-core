#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup

from duckietown_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped


class LocalizationNode(DTROS):

    def __init__(self):
        super(LocalizationNode, self).__init__(
            node_name='localization_node',
            node_type=NodeType.LOCALIZATION
        )
        # get static parameters
        self.robot_configuration = rospy.get_param('~robot_configuration', 'NOT_SET')
        self.map_frame = rospy.get_param('~map_frame', '/world')
        # create publisher
        self._group = DTCommunicationGroup("/localization/tags", TransformStamped)
        self._tags_pub = self._group.Publisher()
        # create subscribers
        self._img_sub = rospy.Subscriber(
            '~detections',
            AprilTagDetectionArray,
            self._cb_detections,
            queue_size=1
        )

    def on_shutdown(self):
        self._group.shutdown()

    def _cb_detections(self, msg):
        for detection in msg.detections:
            out_msg = TransformStamped(
                child_frame_id='/tag/{:s}'.format(str(detection.tag_id)),
                transform=detection.transform
            )
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = self.map_frame
            self._tags_pub.publish(out_msg)


if __name__ == '__main__':
    node = LocalizationNode()
    # spin forever
    rospy.spin()
