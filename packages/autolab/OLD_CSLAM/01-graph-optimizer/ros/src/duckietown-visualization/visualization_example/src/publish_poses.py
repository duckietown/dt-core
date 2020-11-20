#!/usr/bin/env python
import rospy
import tf
import math
import random


def get_random_radius(lower, upper):
    return random.uniform(lower, upper)


def get_random_omega(lower, upper):
    return random.uniform(lower, upper)


def get_random_center(x_lower, x_upper, y_lower, y_upper):
    return (random.uniform(x_lower, x_upper), random.uniform(x_lower, x_upper))


if __name__ == '__main__':

    rospy.init_node('duckiebot_pose_publisher', anonymous=False)

    rate = rospy.Rate(10)  # 10hz

    duckiebot_list = rospy.get_param('~duckiebot_list')

    radius = []
    omega = []
    center = []

    for it in range(len(duckiebot_list)):
        # get a random radius in range [0.3,2] m
        radius.append(get_random_radius(0.3, 2))
        # get a random omega in range [0.1,0.5] rad/s
        omega.append(get_random_omega(0.1, 0.5))
        # get a random center of rotation with x in range [1,5] and y in [2,6]
        center.append(get_random_center(1, 5, 2, 6))

    transform_broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():

        for it in range(len(duckiebot_list)):

            t = rospy.Time.now()
            theta = omega[it]*t.to_sec()
            x = center[it][0] + radius[it]*math.cos(theta)
            y = center[it][1] + radius[it]*math.sin(theta)

            transform_broadcaster.sendTransform((x, y, 0), \
                tf.transformations.quaternion_from_euler(0, 0, theta+math.pi/2), \
                t, duckiebot_list[it], "duckiebot_link")

        rate.sleep()
