#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import numpy as np
import random


def talker():
    pub = rospy.Publisher('/pose_odometry', TransformStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    seq = 0
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        myTransformStamped = TransformStamped()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        s0 = random.randint(0, 5)
        node_type = "duckie"
        h.frame_id = "duckie_88"
        # h.frame_id = "%s_%d" % (node_type, s0)

        myTransformStamped.header = h
        d = random.randint(0, 5)

        # rotation = [random.random(), random.random(),
        #             random.random(), random.random()]
        # rotation = rotation/np.linalg.norm(rotation)
        # rotation = Quaternion(
        #     rotation[0], rotation[1], rotation[2], rotation[3])
        # translation = Vector3(
        #     d - s0, d - s0 + random.random(), d - s0 + random.random())
        node_type = random.choice(["duckie", "watchtower"])

        myTransformStamped.transform = Transform(
            Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        myTransformStamped.child_frame_id = "duckie_88"
        # myTransformStamped.child_frame_id = "%s_%d" % (node_type, d)

        pub.publish(myTransformStamped)

        rate.sleep()
        seq += 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
