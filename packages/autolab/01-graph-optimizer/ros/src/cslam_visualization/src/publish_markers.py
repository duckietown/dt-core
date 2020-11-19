#!/usr/bin/env python3
import rospy
import tf
import cslam_visualization.utils as utils
from visualization_msgs.msg import MarkerArray

if __name__ == '__main__':
    rospy.init_node('cslam_visualization')

    listener = tf.TransformListener()
    pub = rospy.Publisher('cslam_markers', MarkerArray, queue_size=10)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        frames_list = listener.getFrameStrings()

        duckiebots = [
            frame for frame in frames_list if "duckiebot" in frame or "autobot" in frame]
        watchtowers = [frame for frame in frames_list if "watchtower" in frame]
        apriltags = [frame for frame in frames_list if "apriltag" in frame]

        markers = utils.get_markers(
            duckiebots, watchtowers, apriltags, listener)
        pub.publish(markers)

        rate.sleep()
