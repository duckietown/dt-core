#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup

from geometry_msgs.msg import TransformStamped


# TODO: this is temporary
DUCKIEBOT_TAG_ID = 403


class LocalizationNode(DTROS):

    def __init__(self):
        super(LocalizationNode, self).__init__(
            node_name='localization_node',
            node_type=NodeType.LOCALIZATION
        )
        # get static parameters
        self.robot_configuration = rospy.get_param('~robot_configuration', 'NOT_SET')
        self.map_frame = rospy.get_param('~map_frame', '/world')
        # create subscribers
        self._group = DTCommunicationGroup("/localization/tags", TransformStamped)
        self._tags_sub = self._group.Subscriber(self._cb_detections)

    def on_shutdown(self):
        self._group.shutdown()

    def _cb_detections(self, msg, header):
        if msg.child_frame_id == '/tag/{:s}'.format(str(DUCKIEBOT_TAG_ID)):
            print(msg.child_frame_id)
            print(header)


if __name__ == '__main__':
    node = LocalizationNode()
    # spin forever
    rospy.spin()
