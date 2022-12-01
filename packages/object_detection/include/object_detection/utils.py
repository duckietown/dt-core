#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import duckietown_utils as dtu
from geometry_msgs.msg import Point

import os
stop = False

def load_homography():
        """ Load homography from extrinsic parameters """
        robot_name = dtu.get_current_robot_name()
        filename = f"{dtu.get_duckiefleet_root()}/calibrations/camera_extrinsic/{robot_name}.yaml"
        
        if not os.path.isfile(filename):
            rospy.logwarb(f"no extrinsic calibration parameters for {robot_name}, trying default")
            filename = f"{dtu.get_duckiefleet_root()}/calibrations/camera_extrinsic/default.yaml"
            if not os.path.isfile(filename):
                rospy.logerr("Can't find default extrinsic parameters")
            else:
                data = dtu.yaml_wrap.yaml_load_file(filename)
        else:
            # rospy.loginfo("Using extrinsic calibration of " + robot_name)
            data = dtu.yaml_wrap.yaml_load_file(filename)

        #? wrapper load str in byte 
        return np.array(data[b'homography']).reshape((3,3))


def find_position(contours, norm_x, norm_y):
        H = load_homography()
        y_err = 0.075 # Todo: Find why their is a 5cm gap

        positions = []

        for cnt in contours:
            # Extract x, y, width, height
            x,y,w,h = cv2.boundingRect(cnt)
            x_arr, y_arr = np.array([x, x+w]), np.array([y, y+h])

            points = []

            for i in range(len(x_arr)):
                # Normalise
                u = x_arr[i] * 480/norm_x
                v = y_arr[1] * 640/norm_y

                # Find ground projection
                uv_raw = np.array([u, v])
                uv_raw = np.append(uv_raw, np.array([1]))
                ground_point = np.dot(H, uv_raw)
                point = Point()
                x = ground_point[0]
                y = ground_point[1]
                z = ground_point[2]

                # Save in point
                point.x = x/z
                point.y = - y/z + y_err
                point.z = 0.0

                points.append(point)
            
            positions.append((cnt, points))

        return positions
    