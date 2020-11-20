#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from duckietown_visualization.utils import get_list_of_supported_types, \
    get_marker_array_from_map

import contracts
contracts.disable_all()
import duckietown_world as dw


if __name__ == '__main__':

    rospy.init_node('duckietown_map_publisher', anonymous=False)

    pub = rospy.Publisher('duckietown_map', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    map_name = rospy.get_param('~map_name')

    m = dw.load_map(map_name)
    
    rospy.loginfo("Map %s loaded", map_name)

    marker_array = get_marker_array_from_map(m)
    rospy.loginfo("Marker array created. Now publishing!")

    while not rospy.is_shutdown():
        pub.publish(marker_array)
        rate.sleep()
