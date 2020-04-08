#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose

import rospy
from duckietown import DTROS


class VehicleAvoidanceControlNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(VehicleAvoidanceControlNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~desired_distance'] = None
        self.parameters['~minimal_distance'] = None
        self.parameters['~Kp'] = None
        self.parameters['~Ki'] = None
        self.parameters['~Kd'] = None
        self.parameters['~Kp_delta_v'] = None
        self.updateParameters()

        self.controllerInitialization()
        self.detection_prev = None

        # subscribers
        self.subscriber = self.subscriber("~detection", BoolStamped, self.cb_detection, queue_size=1)
        self.sub_vehicle_pose = self.subscriber("~vehicle_pose", VehiclePose, self.cb_pose, queue_size=1)
        self.sub_car_cmd = self.subscriber("~car_cmd_in", Twist2DStamped, self.cb_car_cmd, queue_size=1)

        # publishers
        self.car_cmd_pub = self.publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.vehicle_detected_pub = self.publisher("~vehicle_detected", BoolStamped, queue_size=1)


    def controllerInitialization(self):
        self.vehicle_pose_msg_temp = VehiclePose()
        self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
        self.time_temp = rospy.Time.now()
        self.v_rel = 0
        self.v = 0
        self.detection = False
        self.v_error_temp = 0
        self.I = 0
        self.v_follower = 0
        # self.rho_temp = 0
        self.omega = 0

    def cb_detection(self, data):

        vehicle_detected_msg_out = BoolStamped()
        vehicle_detected_msg_out.header.stamp = data.header.stamp
        vehicle_detected_msg_out.data = data.data
        self.vehicle_detected_pub.publish(vehicle_detected_msg_out)
        self.detection_prev = self.detection
        self.detection = data.data

        if not data.data:
            self.I = 0

    def cb_pose(self, vehicle_pose_msg):
        time = rospy.Time.now()

        Ts = (time - self.time_temp).to_sec()
        self.vehicle_pose_msg_temp.header.stamp = vehicle_pose_msg.header.stamp
        if Ts > 4:
            self.v_rel = 0
            if vehicle_pose_msg.rho.data < self.minimal_distance:
                self.v = 0
            else:
                self.v = self.v_follower
            self.vehicle_pose_msg_temp = vehicle_pose_msg
            self.v_error_temp = 0
            self.I = 0
        else:
            self.v_rel = (vehicle_pose_msg.rho.data - self.vehicle_pose_msg_temp.rho.data) / Ts
            v_leader = self.v_follower + self.v_rel
            delta_v = (vehicle_pose_msg.rho.data - self.desired_distance) / Ts * self.Kp_delta_v
            v_des = v_leader + delta_v
            v_error = v_des - self.v_follower

            self.P = self.Kp * v_error
            self.I = self.I + self.Ki * (v_error + self.v_error_temp) / 2.0 * Ts
            self.D = self.Kd * (v_error + self.v_error_temp) / Ts
            self.v = self.P + self.I + self.D

            if self.v < 0 or vehicle_pose_msg.rho.data < self.minimal_distance:
                self.v = 0

            self.v_error_temp = v_error
            self.v_temp = self.v
            self.vehicle_pose_msg_temp = vehicle_pose_msg

        self.time_temp = time

    def cb_car_cmd(self, car_cmd_msg):
        car_cmd_msg_current = Twist2DStamped()
        car_cmd_msg_current = car_cmd_msg
        car_cmd_msg_current.header.stamp = rospy.Time.now()
        if self.detection:
            car_cmd_msg_current.v = self.v
            if self.v == 0:
                car_cmd_msg_current.omega = 0

        if self.detection_prev and not self.detection:
            car_cmd_msg_current.v = 0

        if car_cmd_msg_current.v >= 0.25:
            car_cmd_msg_current.v = 0.25

        self.car_cmd_pub.publish(car_cmd_msg_current)

if __name__ == '__main__':
    controller = VehicleAvoidanceControlNode('vehicle_avoidance_control_node')
    rospy.spin()
