#!/usr/bin/env python
import rospy
import math
import numpy as np
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState
import time


class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None
        self.last_ms = None
        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_actuator_limits_received = rospy.Publisher("~actuator_limits_received", BoolStamped, queue_size=1)
        self.pub_radius_limit = rospy.Publisher("~radius_limit", BoolStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.PoseHandling, "lane_filter", queue_size=1)
        # self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        
        #TO DO find node/topic in their branch
        self.sub_obstacle_avoidance_pose = rospy.Subscriber("~obstacle_avoidance_pose", LanePose, self.PoseHandling, "obstacle_avoidance",queue_size=1)
        self.sub_obstacle_detected = rospy.Subscriber("~obstacle_detected", BoolStamped, self.setFlag, "obstacle_detected", queue_size=1)
        
        #TO DO find node/topic in their branch
        # self.sub_intersection_navigation_pose = rospy.Subscriber("~intersection_navigation_pose", LanePose, self.PoseHandling, "intersection_navigation",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !
        
        #TO DO find node/topic in their branch
        # self.sub_parking_pose = rospy.Subscriber("~parking_pose", LanePose, self.PoseHandling, "parking",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        #TO DO find node/topic in their branch
        # self.sub_fleet_planning_pose = rospy.Subscriber("~fleet_planning", LanePose, self.PoseHandling, "fleet_planning",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !
        # self.sub_fleet_planning_lane_following_override_active = rospy.Subscriber("~fleet_planning_lane_following_override_active", BoolStamped, self.setFlag, "fleet_planning_lane_following_override_active", queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        #TO DO find node/topic in their branch
        # self.sub_implicit_coord_pose = rospy.Subscriber("~implicit_coord", LanePose, self.PoseHandling, "implicit_coord",queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !
        # self.sub_implicit_coord_velocity_limit_active = rospy.Subscriber("~implicit_coord_velocity_limit_active", BoolStamped, self.setFlag, "implicit_coord_velocity_limit_active", queue_size=1)   # TODO: remap topic in file catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch !

        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.updateWheelsCmdExecuted, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("~actuator_limits", Twist2DStamped, self.updateActuatorLimits, queue_size=1)

        # FSM 
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch,  queue_size=1)     # for this topic, no remapping is required, since it is directly defined in the namespace lane_controller_node by the fsm_node (via it's default.yaml file)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        self.msg_radius_limit = BoolStamped()
        self.msg_radius_limit.data = self.use_radius_limit
        self.pub_radius_limit.publish(self.msg_radius_limit)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        k_Id = 2.5
        k_Iphi = 1.25
        self.cross_track_err = 0
        self.heading_err = 0
        self.cross_track_integral = 0
        self.heading_integral = 0
        self.cross_track_integral_top_cutoff = 0.3
        self.cross_track_integral_bottom_cutoff = -0.3
        self.heading_integral_top_cutoff = 1.2
        self.heading_integral_bottom_cutoff = -1.2
        self.time_start_curve = 0
        use_feedforward_part = False
        self.wheels_cmd_executed = WheelsCmdStamped()

        self.actuator_limits = Twist2DStamped()
        self.actuator_limits.v = 999.0     # to make sure the limit is not hit before the message is received
        self.actuator_limits.omega = 999.0     # to make sure the limit is not hit before the message is received
        self.omega_max = 999.0     # considering radius limitation and actuator limits   # to make sure the limit is not hit before the message is received

        self.use_radius_limit = True

        self.flag_dict = {"obstacle_detected": False,
                            "parking_stop": False,
                            "fleet_planning_lane_following_override_active": False,
                            "implicit_coord_velocity_limit_active": False}

        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.v_ref_possible = dict()
        self.main_pose_source = None

        self.active = True

        # overwrites some of the above set default values (the ones that are already defined in the corresponding yaml-file (see launch-file of this node))
        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        self.k_d = self.setupParameter("~k_d",k_d) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_theta) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",d_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",theta_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

        self.k_Id = self.setupParameter("~k_Id", k_Id)
        self.k_Iphi = self.setupParameter("~k_Iphi",k_Iphi)
        self.use_feedforward_part = self.setupParameter("~use_feedforward_part",use_feedforward_part)
        self.use_radius_limit = self.setupParameter("~use_radius_limit", self.use_radius_limit)
        self.min_radius = self.setupParameter("~min_rad", 0.0)


    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")
        use_radius_limit = rospy.get_param("~use_radius_limit")

        #FeedForward
        self.velocity_to_m_per_s = 0.67 # TODO: change according to information from team System ID!
        self.omega_to_rad_per_s = 0.45
        self.curvature_outer = 1 / (0.39)
        self.curvature_inner = 1 / 0.175
        use_feedforward_part = rospy.get_param("~use_feedforward_part")

        k_Id = rospy.get_param("~k_Id")
        k_Iphi = rospy.get_param("~k_Iphi")
        if self.k_Id != k_Id:
            rospy.loginfo("ADJUSTED I GAIN")
            self.cross_track_integral = 0
            self.k_Id = k_Id
        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset, self.k_Id, self.k_Iphi, self.use_feedforward_part, self.use_radius_limit)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset, k_Id, k_Iphi, use_feedforward_part, use_radius_limit)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.k_Id = k_Id
            self.k_Iphi = k_Iphi
            self.use_feedforward_part = use_feedforward_part

            if use_radius_limit != self.use_radius_limit:
                self.use_radius_limit = use_radius_limit
                self.msg_radius_limit.data = self.use_radius_limit
                self.pub_radius_limit.publish(self.msg_radius_limit)


    # FSM
    def cbSwitch(self,fsm_switch_msg):
        self.active = fsm_switch_msg.data # True or False
        rospy.loginfo("active: " + str(self.active))


    # FSM
    def cbMode(self,fsm_state_msg):
        self.fsm_state = fsm_state_msg.state # String of current FSM state
        print "fsm_state changed in lane_controller_node to: " , self.fsm_state


    def setFlag(self, msg_flag, flag_name):
        self.flag_dict[flag_name] = msg_flag.data
        
    
    def PoseHandling(self, input_pose_msg, pose_source):
        if not self.active:
            return

        self.prev_pose_msg = self.pose_msg
        self.pose_msg_dict[pose_source] = input_pose_msg
        if self.pose_initialized:
            v_ref_possible_main_pose = self.v_ref_possible["main_pose"]
            self.v_ref_possible.clear()
            self.v_ref_possible["main_pose"] = v_ref_possible_main_pose

        if self.fsm_state == "INTERSECTION_CONTROL":
            if pose_source == "intersection_navigation":
                self.pose_msg = input_pose_msg
                self.v_ref_possible["main_pose"] = input_pose_msg.v_ref
                self.main_pose_source = pose_source
                self.pose_initialized = True
        elif self.fsm_state == "PARKING":
            if pose_source == "parking":
                self.pose_msg = input_pose_msg
                self.v_ref_possible["main_pose"] = input_pose_msg.v_ref
                self.main_pose_source = pose_source
                self.pose_initialized = True
        else:
            if pose_source == "lane_filter":
                self.pose_msg = input_pose_msg
                self.pose_msg.curvature_ref = input_pose_msg.curvature
                self.v_ref_possible["main_pose"] = self.v_bar
                self.main_pose_source = pose_source
                self.pose_initialized = True

        if self.flag_dict["fleet_planning_lane_following_override_active"] == True:
            if "fleet_planning" in self.pose_msg_dict:
                self.pose_msg.d_ref = self.pose_msg_dict["fleet_planning"].d_ref
                self.v_ref_possible["fleet_planning"] = self.pose_msg_dict["fleet_planning"].v_ref
        if self.flag_dict["obstacle_detected"] == True:
            if "obstacle_avoidance" in self.pose_msg_dict:
                self.pose_msg.d_ref = self.pose_msg_dict["obstacle_avoidance"].d_ref
                self.v_ref_possible["obstacle_avoidance"] = self.pose_msg_dict["obstacle_avoidance"].v_ref
        if self.flag_dict["implicit_coord_velocity_limit_active"] == True:
            if "implicit_coord" in self.pose_msg_dict:
                self.v_ref_possible["implicit_coord"] = self.pose_msg_dict["implicit_coord"].v_ref

        self.pose_msg.v_ref = min(self.v_ref_possible.itervalues())

        if self.pose_msg != self.prev_pose_msg and self.pose_initialized:
            self.cbPose(self.pose_msg)


    def updateWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd


    def updateActuatorLimits(self, msg_actuator_limits):
        self.actuator_limits = msg_actuator_limits
        rospy.loginfo("actuator limits updated to: ")
        rospy.loginfo("actuator_limits.v: " + str(self.actuator_limits.v))
        rospy.loginfo("actuator_limits.omega: " + str(self.actuator_limits.omega))
        msg_actuator_limits_received = BoolStamped()
        msg_actuator_limits_received.data = True
        self.pub_actuator_limits_received.publish(msg_actuator_limits_received)


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


    def cbPose(self, pose_msg):
        self.lane_reading = pose_msg
        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        prev_cross_track_err = self.cross_track_err
        prev_heading_err = self.heading_err
        self.cross_track_err = pose_msg.d - self.d_offset
        self.heading_err = pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header
        car_control_msg.v = pose_msg.v_ref #*self.speed_gain #Left stick V-axis. Up is positive

        if math.fabs(self.cross_track_err) > self.d_thres:
            rospy.logerr("inside threshold ")
            self.cross_track_err = self.cross_track_err / math.fabs(self.cross_track_err) * self.d_thres

        currentMillis = int(round(time.time() * 1000))

        if self.last_ms is not None:
            dt = (currentMillis - self.last_ms) / 1000.0
            self.cross_track_integral += self.cross_track_err * dt
            self.heading_integral += self.heading_err * dt

        if self.cross_track_integral > self.cross_track_integral_top_cutoff:
            self.cross_track_integral = self.cross_track_integral_top_cutoff
        if self.cross_track_integral < self.cross_track_integral_bottom_cutoff:
            self.cross_track_integral = self.cross_track_integral_bottom_cutoff

        if self.heading_integral > self.heading_integral_top_cutoff:
            self.heading_integral = self.heading_integral_top_cutoff
        if self.heading_integral < self.heading_integral_bottom_cutoff:
            self.heading_integral = self.heading_integral_bottom_cutoff

        if abs(self.cross_track_err) <= 0.011:       # TODO: replace '<= 0.011' by '< delta_d' (but delta_d might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.cross_track_integral = 0
        if abs(self.heading_err) <= 0.051:           # TODO: replace '<= 0.051' by '< delta_phi' (but delta_phi might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.heading_integral = 0
        if np.sign(self.cross_track_err) != np.sign(prev_cross_track_err):      # sign of error changed => error passed zero
            self.cross_track_integral = 0
        if np.sign(self.heading_err) != np.sign(prev_heading_err):      # sign of error changed => error passed zero
            self.heading_integral = 0
        if self.wheels_cmd_executed.vel_right == 0 and self.wheels_cmd_executed.vel_left == 0:    # if actual velocity sent to the motors is zero
            self.cross_track_integral = 0
            self.heading_integral = 0

        omega_feedforward = self.v_bar * self.velocity_to_m_per_s * pose_msg.curvature_ref * 2 * math.pi
        if self.main_pose_source == "lane_filter" and not self.use_feedforward_part:
            omega_feedforward = 0

        omega =  self.k_d * self.cross_track_err + self.k_theta * self.heading_err
        omega -= self.k_Id * self.cross_track_integral
        omega -= self.k_Iphi * self.heading_integral
        omega +=  ( omega_feedforward) * self.omega_to_rad_per_s

        self.omega_max_radius_limitation = self.v_bar * self.velocity_to_m_per_s / self.min_radius * 2 * math.pi * self.omega_to_rad_per_s
        self.omega_max = min(self.actuator_limits.omega, self.omega_max_radius_limitation)

        if omega > self.omega_max:
            self.cross_track_integral -= self.cross_track_err * dt
            self.heading_integral -= self.heading_err * dt
            car_control_msg.omega = self.omega_max
        else:
            car_control_msg.omega = omega

        if car_control_msg.v > self.actuator_limits.v:
            car_control_msg.v = self.actuator_limits.v

        rospy.loginfo("pose_msg.curvature_ref: " + str(pose_msg.curvature_ref))
        rospy.loginfo("heading_err: " + str(self.heading_err))
        rospy.loginfo("heading_integral: " + str(self.heading_integral))
        rospy.loginfo("cross_track_err: " + str(self.cross_track_err))
        rospy.loginfo("cross_track_integral: " + str(self.cross_track_integral))
        rospy.loginfo("use_feedforward_part: " + str(self.use_feedforward_part))
        rospy.loginfo("fsm_state in lane_controller_node: " + str(self.fsm_state))
        self.publishCmd(car_control_msg)
        self.last_ms = currentMillis


if __name__ == "__main__":
    rospy.init_node("lane_controller_node",anonymous=False) #adapted to sonjas default file
    lane_control_node = lane_controller()
    rospy.spin()
