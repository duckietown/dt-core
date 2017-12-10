#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose
####JULIEN from duckietown_msgs.msg import LaneCurvature
class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        #####JULIEN self.sub_curvature = rospy.Subscriber("~curvature", LaneCurvature, self.cbCurve, queue_size=1)
        #####JULIEN self.k_forward = 0.0
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value



    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        incurvature = False
        curve_inner = False
        k_Id = 0.2
        k_Iphi = 0.1
        self.cross_track_integral = 0
        self.heading_integral = 0
        self.time_start_curve = 0

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        # FIXME: AC aug'17: are these inverted?
        self.k_d = self.setupParameter("~k_d",k_d) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_theta) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",d_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",theta_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

        self.k_Id = self.setupParameter("~k_Id", k_Id)
        self.k_Iphi = self.setupParameter("~k_Iphi",k_Iphi)
        self.incurvature = self.setupParameter("~incurvature",incurvature)
        self.curve_inner = self.setupParameter("~curve_inner",curve_inner)

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        #FeedForward
        self.velocity_to_m_per_s = 0.67 # TODO: change according to information from team System ID!
        self.omega_to_rad_per_s = 0.45
        self.curvature_outer = 1 / (0.39)
        self.curvature_inner = 1 / 0.175
        incurvature = rospy.get_param("~incurvature") # TODO remove after estimator is introduced
        curve_inner = rospy.get_param("~curve_inner") # TODO remove after estimator is introduced

        k_Id = rospy.get_param("~k_Id")
        k_Iphi = rospy.get_param("~k_Iphi")
        if self.k_Id != k_Id:
            rospy.loginfo("ADJUSTED I GAIN")
            self.cross_track_integral = 0
            self.k_Id = k_Id
        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset, self.k_Id, self.k_Iphi, self.incurvature, self.curve_inner)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset, k_Id, k_Iphi, incurvature, curve_inner)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            #rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            #rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.k_Id = k_Id
            self.k_Iphi = k_Iphi

            if incurvature != self.incurvature and incurvature:
                self.time_start_curve = rospy.Time.now().secs

            self.incurvature = incurvature
            self.curve_inner = curve_inner




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

    def cbPose(self, lane_pose_msg):

        self.lane_reading = lane_pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive

        if math.fabs(cross_track_err) > self.d_thres:
            rospy.logerr("inside threshold ")
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres

        if self.cross_track_integral > 4:
            rospy.loginfo("you're greater 5")
            self.cross_track_integral = 4
        if self.cross_track_integral < -4:
            rospy.loginfo("youre smaller -5")
            self.cross_track_integral = -4

        self.cross_track_integral += cross_track_err
        self.heading_integral += heading_err

        if self.heading_integral < -15:
            self.heading_integral = -15
        if self.heading_integral > 15:
            self.heading_integral = 15

        if self.curve_inner:
            self.curvature  = self.curvature_inner
        else:
            self.curvature = self.curvature_outer
        omega_feedforward = self.v_bar * self.velocity_to_m_per_s * self.curvature * 2 * math.pi

        car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err
        rospy.loginfo("P-Control: " + str(car_control_msg.omega))
        rospy.loginfo("Adjustment: " + str(-self.k_Id * self.cross_track_integral))

        if not self.incurvature:
            if heading_err > 0.3:
                self.incurvature = True
                rospy.set_param('~incurvature',True)
            car_control_msg.omega -= self.k_Id * self.cross_track_integral
            car_control_msg.omega -= self.k_Iphi * self.heading_integral #*self.steer_gain #Right stick H-axis. Right is negative
        else:
            if self.curve_inner:
                time_incurve = 1
            else:
                time_incurve = 3
            if (rospy.Time.now().secs - self.time_start_curve) > time_incurve:   #TODO fix 5 to a time in curvature with v and d
                rospy.set_param('~incurvature',False)
                self.incurvature = False
            rospy.loginfo("incurvature : ")
            car_control_msg.omega +=  ( omega_feedforward) * self.omega_to_rad_per_s
        rospy.loginfo("kid : " + str(self.k_Id))
        rospy.loginfo("Kd : " + str(self.k_d))
        #rospy.loginfo("k_Iphi * heading : " + str(self.k_Iphi * self.heading_integral))
        rospy.loginfo("k_Iphi :" + str(self.k_Iphi))
        rospy.loginfo("Ktheta : " + str(self.k_theta))
        rospy.loginfo("incurvature : " + str(self.incurvature))
        rospy.loginfo("cross_track_err : " + str(cross_track_err))
        rospy.loginfo("heading_err : " + str(heading_err))
        #rospy.loginfo("Ktheta : Versicherung")

        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(car_control_msg)

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
