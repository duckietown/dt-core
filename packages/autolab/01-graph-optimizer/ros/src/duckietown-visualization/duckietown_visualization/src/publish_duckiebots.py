#!/usr/bin/env python  
import rospy
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray

def get_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/duckiebot_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker

if __name__ == '__main__':
    rospy.init_node('duckiebot_marker_publisher')

    listener = tf.TransformListener()

    duckiebot_list = rospy.get_param('~duckiebot_list')
    pub = rospy.Publisher('duckiebots_markers', MarkerArray, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        for it in range(len(duckiebot_list)): 
            try:
                (trans,rot) = listener.lookupTransform('/duckiebot_link', \
                    duckiebot_list[it], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException):
                continue
            marker_array.markers.append(get_marker(it,trans[0],trans[1],rot))
        
        pub.publish(marker_array)


        rate.sleep()
