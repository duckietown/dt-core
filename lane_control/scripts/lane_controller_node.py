#!/usr/bin/env python
import math
import time

from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, ActuatorParameters, BoolStamped
import numpy as np
import rospy


####JULIEN from duckietown_msgs.msg import LaneCurvature
class lane_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None
        self.last_ms = None
        self.pub_counter = 0

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_actuator_params_received = rospy.Publisher("~actuator_params_received", BoolStamped, queue_size=1)
        self.pub_radius_limit = rospy.Publisher("~radius_limit", BoolStamped, queue_size=1)

        # Subscription
        # TODO: set normal_use False for using your topic
        normal_use = True
        if normal_use:
            self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        else:
            # TODO: add your own subscriber here by modifying topic, choose from lane_pose_obstacle_avoidance,lane_pose_parking, implicit_coordination_velocity,lane_pose_intersection_navigation
            #self.sub_lane_reading = rospy.Subscriber("~lane_pose_intersection_navigation", LanePose, self.cbPose, queue_size=1)
            self.sub_lane_reading = rospy.Subscriber("~yourtopic", LanePose, self.cbPose, queue_size=1)

        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.updateWheelsCmdExecuted, queue_size=1)
        self.sub_actuator_params = rospy.Subscriber("~actuator_params", ActuatorParameters, self.updateActuatorParameters, queue_size=1)

        robot_name = rospy.get_param("~veh", "")  # ToDo Controllers: potentially update param to get the robot name
        self.sub_topic = '/{}/obstacle_avoidance_active_flag'.format(robot_name)
        #self.subscriber = rospy.Subscriber(self.sub_topic, bool, self.obstacleFlagCallback)

        #self.sub_topic = '/{}/obst_avoid/d_target'.format(robot_name)
        #self.subscriber = rospy.Subscriber(self.sub_topic, Float32,
        #                                   self.obstAvoidDUpdate)  # Absolute value, with 0 on center line

        #self.sub_topic = '/obstacle_emergency_stop_flag'.format(robot_name)
        #self.subscriber = rospy.Subscriber(self.sub_topic, Bool, self.obstEmergBrakeUpdate)

        #####JULIEN self.sub_curvature = rospy.Subscriber("~curvature", LaneCurvature, self.cbCurve, queue_size=1)
        #####JULIEN self.k_forward = 0.0

        # Setup parameters
        self.setGains()

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def obstacleFlagCallback(self, obstacleFlagUpdate):
        self.object_detected = obstacleFlagUpdate

    def obstAvoidDUpdate(self, dUpdate):
        self.d_ref = dUpdate

    def obstEmergBrakeUpdate(self, brakeUpdate):
        #ToDo: Emergency Brake Flag
        # self.emergencybrake = brakeUpdate
        pass

    def setGains(self):
        v_bar = 0.5  # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = -(k_theta ** 2) / (4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0
        d_ref = 0.0
        phi_ref = 0.0
        object_detected = False

        # incurvature = False
        # curve_inner = False
        k_Id = 2.5
        k_Iphi = 1.25
        self.cross_track_err = 0
        self.heading_err = 0
        self.cross_track_integral = 0
        self.heading_integral = 0
        self.time_start_curve = 0
        turn_off_feedforward_part = True
        self.wheels_cmd_executed = WheelsCmdStamped()

        self.actuator_params = ActuatorParameters()
        self.actuator_params.gain = 0.0
        self.actuator_params.trim = 0.0
        self.actuator_params.baseline = 0.0
        self.actuator_params.radius = 0.0
        self.actuator_params.k = 0.0
        self.actuator_params.limit = 0.0
        self.omega_max = 999.0  # TODO: change!

        self.use_radius_limit = True

        # overwrites some of the above set default values (the ones that are already defined in the corresponding yaml-file)
        self.v_bar = self.setupParameter("~v_bar", v_bar)  # Linear velocity
        # FIXME: AC aug'17: are these inverted?
        self.k_d = self.setupParameter("~k_d", k_d)  # P gain for theta
        self.k_theta = self.setupParameter("~k_theta", k_theta)  # P gain for d
        self.d_thres = self.setupParameter("~d_thres", d_thres)  # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres", theta_thres)  # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset", d_offset)  # a configurable offset from the lane position
        self.d_ref = self.setupParameter("~d_ref", d_ref)
        self.phi_ref = self.setupParameter("~phi_ref", phi_ref)
        self.object_detected = self.setupParameter("~object_detected", object_detected)  # object detected flag

        self.k_Id = self.setupParameter("~k_Id", k_Id)
        self.k_Iphi = self.setupParameter("~k_Iphi", k_Iphi)
        self.turn_off_feedforward_part = self.setupParameter("~turn_off_feedforward_part", turn_off_feedforward_part)
        # self.incurvature = self.setupParameter("~incurvature",incurvature)
        # self.curve_inner = self.setupParameter("~curve_inner",curve_inner)
        self.use_radius_limit = self.setupParameter("~use_radius_limit", self.use_radius_limit)

        self.msg_radius_limit = BoolStamped()
        self.msg_radius_limit.data = self.use_radius_limit
        self.pub_radius_limit.publish(self.msg_radius_limit)

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")
        d_ref = rospy.get_param("~d_ref")
        phi_ref = rospy.get_param("~phi_ref")
        use_radius_limit = rospy.get_param("~use_radius_limit")
        object_detected = rospy.get_param("~object_detected")

        #FeedForward
        self.velocity_to_m_per_s = 0.67  # TODO: change according to information from team System ID!
        self.omega_to_rad_per_s = 0.45
        self.curvature_outer = 1 / (0.39)
        self.curvature_inner = 1 / 0.175
        # incurvature = rospy.get_param("~incurvature") # TODO remove after estimator is introduced
        # curve_inner = rospy.get_param("~curve_inner") # TODO remove after estimator is introduced
        turn_off_feedforward_part = rospy.get_param("~turn_off_feedforward_part")

        k_Id = rospy.get_param("~k_Id")
        k_Iphi = rospy.get_param("~k_Iphi")
        if self.k_Id != k_Id:
            rospy.loginfo("ADJUSTED I GAIN")
            self.cross_track_integral = 0
            self.k_Id = k_Id
        params_old = (self.v_bar, self.k_d, self.k_theta, self.d_thres, self.theta_thres, self.d_offset, self.d_ref, self.phi_ref, self.k_Id, self.k_Iphi, self.turn_off_feedforward_part, self.use_radius_limit, self.object_detected)
        params_new = (v_bar, k_d, k_theta, d_thres, theta_thres, d_offset, d_ref, phi_ref, k_Id, k_Iphi, turn_off_feedforward_part, use_radius_limit, object_detected)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." % (self.node_name))
            #rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            #rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.d_ref = d_ref
            self.phi_ref = phi_ref
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.k_Id = k_Id
            self.k_Iphi = k_Iphi
            self.turn_off_feedforward_part = turn_off_feedforward_part
            self.object_detected = object_detected

            if use_radius_limit != self.use_radius_limit:
                self.use_radius_limit = use_radius_limit
                self.msg_radius_limit.data = self.use_radius_limit
                self.pub_radius_limit.publish(self.msg_radius_limit)

            # if incurvature != self.incurvature and incurvature:
            #     self.time_start_curve = rospy.Time.now().secs

            # self.incurvature = incurvature
            # self.curve_inner = curve_inner

    def updateWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def updateActuatorParameters(self, msg_actuator_params):
        self.actuator_params = msg_actuator_params
        rospy.loginfo("actuator_params updated to: ")
        rospy.loginfo("actuator_params.gain: " + str(self.actuator_params.gain))
        rospy.loginfo("actuator_params.trim: " + str(self.actuator_params.trim))
        rospy.loginfo("actuator_params.baseline: " + str(self.actuator_params.baseline))
        rospy.loginfo("actuator_params.radius: " + str(self.actuator_params.radius))
        rospy.loginfo("actuator_params.k: " + str(self.actuator_params.k))
        rospy.loginfo("actuator_params.limit: " + str(self.actuator_params.limit))
        msg_actuator_params_received = BoolStamped()
        msg_actuator_params_received.data = True
        self.pub_actuator_params_received.publish(msg_actuator_params_received)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5)  #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

    def publishCmd(self, car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    ##JULIEN
    #def cbCurve(self, msg):
    #    curvetype = msg.curvetype
    #    if curvetype == LaneCurvature.LEFT:
    #        self.k_forward = ...
    #    if curvetype == LaneCurvature.RIGHT:
    #        self.k_forward = ...
    #    if curvetype == LaneCurvature.STRAIGHT:
    #        self.k_forward = 0.0

    ###TODO SAVIORS

    def cbPose(self, lane_pose_msg):
        self.lane_reading = lane_pose_msg

       # self.d_target_pub = rospy.Publisher(self.pub_topic, Float32, queue_size=1)

        #if self.object_detected:   # if object is detected (TRUE)
            #self.d_ref = 0.02 # set d_ref here, LanePose message from saviors
            # negetive value: moves towards right line
            # positive value: moves towards left line

        ###END TODO

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs / 1e9

        prev_cross_track_err = self.cross_track_err
        prev_heading_err = self.heading_err
        self.cross_track_err = lane_pose_msg.d - self.d_offset - self.d_ref
        self.heading_err = lane_pose_msg.phi - self.phi_ref

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar  #*self.speed_gain #Left stick V-axis. Up is positive

        if math.fabs(self.cross_track_err) > self.d_thres:
            rospy.logerr("inside threshold ")
            self.cross_track_err = self.cross_track_err / math.fabs(self.cross_track_err) * self.d_thres

        currentMillis = int(round(time.time() * 1000))

        if self.last_ms is not None:
            dt = (currentMillis - self.last_ms) / 1000.0
            self.cross_track_integral += self.cross_track_err * dt
            self.heading_integral += self.heading_err * dt

        if self.cross_track_integral > 0.3:
            rospy.loginfo("you're greater 0.3")
            self.cross_track_integral = 0.3
        if self.cross_track_integral < -0.3:
            rospy.loginfo("youre smaller -0.3")
            self.cross_track_integral = -0.3

        if self.heading_integral < -1.2:
            self.heading_integral = -1.2
        if self.heading_integral > 1.2:
            self.heading_integral = 1.2

        if abs(self.cross_track_err) <= 0.011:  # TODO: replace '<= 0.011' by '< delta_d' (but delta_d might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.cross_track_integral = 0
        if abs(self.heading_err) <= 0.051:  # TODO: replace '<= 0.051' by '< delta_phi' (but delta_phi might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.heading_integral = 0
        if np.sign(self.cross_track_err) != np.sign(prev_cross_track_err):
            self.cross_track_integral = 0
        if np.sign(self.heading_err) != np.sign(prev_heading_err):
            self.heading_integral = 0
        if self.wheels_cmd_executed.vel_right == 0 and self.wheels_cmd_executed.vel_left == 0:
            self.cross_track_integral = 0
            self.heading_integral = 0

        # if velocity_of_actual_motor_comand == 0:       # TODO: get this velocity that is actually sent to the motors and plug in here
        #     self.cross_track_integral = 0
        #     self.heading_integral = 0

        # if self.curve_inner:
        #     self.curvature  = self.curvature_inner
        # else:
        #     self.curvature = self.curvature_outer
        omega_feedforward = self.v_bar * self.velocity_to_m_per_s * lane_pose_msg.curvature * 2 * math.pi
        if self.turn_off_feedforward_part:
            omega_feedforward = 0

        omega = self.k_d * self.cross_track_err + self.k_theta * self.heading_err
        # rospy.loginfo("P-Control: " + str(car_control_msg.omega))
        # rospy.loginfo("Adjustment: " + str(-self.k_Id * self.cross_track_integral))
        omega -= self.k_Id * self.cross_track_integral
        omega -= self.k_Iphi * self.heading_integral
        omega += (omega_feedforward) * self.omega_to_rad_per_s

        ### omega_max_actuator_params = .....  # TODO: complete (based on parameters from self.actuator_params)
        ### omega_max_radius_limitation = .....  # TODO: complete (based on radius limitation)
        ### self.omega_max = min(omega_max_actuator_params, omega_max_radius_limitation)

        if omega > self.omega_max:
            self.cross_track_integral -= self.cross_track_err * dt
            self.heading_integral -= self.heading_err * dt
            ### if omega > omega_max_radius_limitation:
            ###     car_control_msg.omega = omega_max_radius_limitation
        else:
            car_control_msg.omega = omega

        # if not self.incurvature:
        #     if self.heading_err > 0.3:
        #         self.incurvature = True
        #         rospy.set_param('~incurvature',True)
        #     car_control_msg.omega -= self.k_Id * self.cross_track_integral
        #     car_control_msg.omega -= self.k_Iphi * self.heading_integral #*self.steer_gain #Right stick H-axis. Right is negative
        # else:
        #     if self.curve_inner:
        #         time_incurve = 1
        #     else:
        #         time_incurve = 3
        #     if (rospy.Time.now().secs - self.time_start_curve) > time_incurve:   #TODO fix 5 to a time in curvature with v and d
        #         rospy.set_param('~incurvature',False)
        #         self.incurvature = False
        #     rospy.loginfo("incurvature : ")
        #     car_control_msg.omega +=  ( omega_feedforward) * self.omega_to_rad_per_s
        # rospy.loginfo("kid : " + str(self.k_Id))
        # rospy.loginfo("Kd : " + str(self.k_d))
        #rospy.loginfo("k_Iphi * heading : " + str(self.k_Iphi * self.heading_integral))
        # rospy.loginfo("k_Iphi :" + str(self.k_Iphi))
        # rospy.loginfo("Ktheta : " + str(self.k_theta))
        # rospy.loginfo("incurvature : " + str(self.incurvature))
        # rospy.loginfo("cross_track_err : " + str(self.cross_track_err))
        # rospy.loginfo("heading_err : " + str(self.heading_err))
        #rospy.loginfo("Ktheta : Versicherung")
        rospy.loginfo("lane_pose_msg.curvature: " + str(lane_pose_msg.curvature))
        rospy.loginfo("heading_err: " + str(self.heading_err))
        rospy.loginfo("heading_integral: " + str(self.heading_integral))
        rospy.loginfo("cross_track_err: " + str(self.cross_track_err))
        rospy.loginfo("cross_track_integral: " + str(self.cross_track_integral))
        rospy.loginfo("turn_off_feedforward_part: " + str(self.turn_off_feedforward_part))
        # rospy.loginfo("actuator_params.gain: " + str(self.actuator_params.gain))
        # rospy.loginfo("actuator_params.trim: " + str(self.actuator_params.trim))
        # rospy.loginfo("actuator_params.baseline: " + str(self.actuator_params.baseline))
        # rospy.loginfo("actuator_params.radius: " + str(self.actuator_params.radius))
        # rospy.loginfo("actuator_params.k: " + str(self.actuator_params.k))
        # rospy.loginfo("actuator_params.limit: " + str(self.actuator_params.limit))

        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(car_control_msg)
        self.last_ms = currentMillis

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg


if __name__ == "__main__":
    rospy.init_node("lane_controller", anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
