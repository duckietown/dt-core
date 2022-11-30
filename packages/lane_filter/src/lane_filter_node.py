#!/usr/bin/env python3
import rospy 


if __name__ == "__main__":
    rospy.init_node("lane_filter_node")
    rospy.loginfo("lane_filter_node")
    rospy.spin()
 
