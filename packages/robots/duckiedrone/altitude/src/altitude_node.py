#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty

from duckietown.dtros import DTROS, NodeType


class AltitudeNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AltitudeNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        # parameters
        self._h_offset = rospy.get_param("~h_offset", 0.0)

        # this is the angle between the direction the range finder is pointed (normal to the drone)
        # and the gravity vector (towards the ground)
        self._angle = 0

        # we keep track of the last range to properly handle out-of-range messages
        self._last_range = 0

        # subscribers
        self._sub_imu = rospy.Subscriber('~imu', Imu, self.imu_cb, queue_size=1)
        self._sub_tof = rospy.Subscriber('~tof', Range, self.tof_cb, queue_size=1)

        # publishers
        self._pub = rospy.Publisher('~altitude', Range, queue_size=1)
        self._heartbeat = rospy.Publisher('~heartbeat', Empty, queue_size=1)

    def imu_cb(self, msg):
        """
        The IMU gives a quaternion `q` and by transforming the gravity vector
        `g=[0,0,1,0]` in quaternion space, using `g' = q dot g dot q_conjugate` you get
        `g'` which is the vector going upwards out of the plane of the body of the drone
        and using `arctan(g dot g')` we are able to find the angle that the drone makes
        with gravity, and therefore what factor to multiply the rangefinder data to convert
        it to distance off the ground.

        Args:
            msg:  the message from the IMU

        """
        g = np.array([0, 0, 1, 0])
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        q1 = np.array(
            [-msg.orientation.x, -msg.orientation.y, -msg.orientation.z, msg.orientation.w])
        g1 = tf.transformations.quaternion_multiply(
            q,
            tf.transformations.quaternion_multiply(
                g,
                q1,
            ),
        )
        self._angle = np.arccos(np.dot(g1[0:3], g[0:3]))

    def tof_cb(self, msg):
        """
        The Time-of-Flight sensor gives the distance to the closest object below the robot.

        Args:
            msg:  the message from the Time-of-Flight sensor

        """
        range = msg.range
        if range > msg.max_range:
            if self._last_range == 0:
                return
            # out-of-range, assume last range
            range = self._last_range
        # keep track of last range
        self._last_range = range
        # offset the range by the distance between the sensor frame and the footprint frame
        range = max(range - self._h_offset, 0.0)
        # compute altitude from range and angle
        altitude = range * np.cos(self._angle)
        # publish message
        msg = Range(
            header=msg.header,
            radiation_type=msg.radiation_type,
            field_of_view=msg.field_of_view,
            min_range=msg.min_range,
            max_range=msg.max_range,
            range=altitude
        )
        self._pub.publish(msg)
        # update heartbeat
        self._heartbeat.publish(Empty())


if __name__ == "__main__":
    range_finder_node = AltitudeNode("altitude_node")
    rospy.spin()
