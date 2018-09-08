#!/usr/bin/env python
import rospy
import cv2
from path_planner.path_planner import PathPlanner
from pose_estimator.pose_estimator import PoseEstimator, VehicleCommand
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AprilTagsWithInfos, FSMState, TagInfo, Twist2DStamped, BoolStamped, IntersectionPose, IntersectionPoseImg, LanePose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String
import duckietown_utils as dt_utils
import numpy as np
import math

class IntersectionNavigation(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))
        self.active = True

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up path planner, state estimator and localizer
        self.intersectionLocalizer = IntersectionLocalizer(self.veh)
        self.pathPlanner = PathPlanner(self.veh)
        self.poseEstimator = PoseEstimator()

        # open-loop / closed-loop
        # Open-loop: Path is tracked by its parametrization s.
        # CLosed-loop: Path is tracked by the controller designed by devel-controllers group. However the controller
        # has been adapted by us. Check lane-controller node.
        # If true remap in 00-infrastructure/duckietown/config/baseline/dagu_car/car_cmd_switch_node/default.yaml
        self.open_loop = False

        # main logic parameters
        self.rate = 10  # main logic runs at 10Hz
        self.timeout = 1.0

        # fsm
        self.go = False

        # turn type
        self.turn_type = -1
        self.turn_type_time = rospy.Time()
        self.turn_type_timeout = 10.0

        # in lane
        self.in_lane = False
        self.in_lane_time = rospy.Time()
        self.in_lane_timeout = 0.5
        self.in_lane_wait_time = 1.0

        self.state_dict = dict()
        for counter, key in enumerate(['IDLE',
                                       'INITIALIZING_LOCALIZATION',
                                       'INITIALIZING_PATH',
                                       'TRAVERSING',
                                       'DONE',
                                       'ERROR']):
            self.state_dict.update({key: counter})
        self.state = self.state_dict['IDLE']

        # auxiliary variables
        self.tag_info = TagInfo()
        self.intersection_signs = [self.tag_info.FOUR_WAY, self.tag_info.RIGHT_T_INTERSECT,
                                   self.tag_info.LEFT_T_INTERSECT, self.tag_info.T_INTERSECTION]
        self.intersection_type = 0 # 0: three way intersection, 1: four way intersection
        self.current_tag_info = None

        # Velocities conversion parameters
        self.v_scale = 1.53
        self.omega_scale = 4.75

        # Set constant velocity
        if self.open_loop:
            self.v = 0.1
        else:
            self.v = 0.2 #TODO was 0.2 before

        # nominal stop positions: centered in lane, 0.16m in front of center of red stop line,
        # 0 relative orientation error
        self.nominal_start_positions = {self.tag_info.FOUR_WAY: [0.400, -0.135, 0.5 * np.pi],
                                       self.tag_info.LEFT_T_INTERSECT: [0.694, 0.400, np.pi],
                                       self.tag_info.RIGHT_T_INTERSECT: [-0.135, 0.121, 0.0 * np.pi],
                                       self.tag_info.T_INTERSECTION: [0.400, -0.135, 0.5 * np.pi]}
        self.nominal_final_positions = [[0.159, 0.0508, -0.5 * np.pi],
                                        [0.508, 0.159, 0.0],
                                        [0.400, 0.508, 0.5 * np.pi],
                                        [0.0508, 0.400, np.pi]]

        # set up subscribers
        self.sub_fsm = rospy.Subscriber("~fsm",
                                        FSMState,
                                        self.FSMCallback,
                                        queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type",
                                              Int16,
                                              self.TurnTypeCallback,
                                              queue_size=1)
        self.sub_img = rospy.Subscriber("~img",
                                        CompressedImage,
                                        self.ImageCallback,
                                        queue_size=1,
                                        buff_size=2**24)
        self.sub_pose = rospy.Subscriber("~pose_in",
                                         IntersectionPose,
                                         self.PoseCallback,
                                         queue_size=1)
        self.sub_cmd = rospy.Subscriber("~cmds",
                                        Twist2DStamped,
                                        self.CmdCallback,
                                        queue_size=30)
        self.sub_in_lane = rospy.Subscriber("~in_lane",
                                        BoolStamped,
                                        self.InLaneCallback,
                                        queue_size=1)

        # set up publishers
        self.pub_intersection_pose_img = rospy.Publisher("~pose_img_out", IntersectionPoseImg, queue_size=1)
        self.pub_lane_pose = rospy.Publisher("~intersection_navigation_pose", LanePose, queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        # Needed if open loop
        self.pub_cmds = rospy.Publisher("~cmds_out", Twist2DStamped, queue_size=1)
        self.pub_path_computed = rospy.Publisher("~path_computed", BoolStamped, queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

        rospy.set_param("~v_inters", 0.2)
        rospy.set_param("~s_max", 0.5)

        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParameter)

    def updateParameter(self, event):
        self.v = rospy.get_param("~v_inters")
        self.s_max = rospy.get_param("~s_max")

    def SelfReset(self):
        self.go = False
        self.turn_type = -1
        self.intersection_type = 0
        self.in_lane = False
        self.state = self.state_dict['IDLE']


    def ComputeFinalPose(self, intersection_type, turn_type):
        if intersection_type == self.tag_info.FOUR_WAY:
            if turn_type == 0: # left
                return self.nominal_final_positions[3]
            elif turn_type == 1: # straight
                return self.nominal_final_positions[2]
            else: # right
                return self.nominal_final_positions[1]

        elif intersection_type == self.tag_info.T_INTERSECTION:
            if turn_type == 0: # left
                return self.nominal_final_positions[3]
            else: # right
                return self.nominal_final_positions[1]

        elif intersection_type == self.tag_info.LEFT_T_INTERSECT:
            if turn_type == 0: # left
                return self.nominal_final_positions[0]
            else: # straight
                return self.nominal_final_positions[3]

        else: # RIGHT_T_INTERSECT:
            if turn_type == 1: # straight
                return self.nominal_final_positions[1]
            else: # right
                return self.nominal_final_positions[0]


    def MainLoop(self):

        if not self.active:
            return
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # run state machine

            if self.state == self.state_dict['IDLE']:
                # waiting for FSMState to tell us that Duckiebot is at an intersection (see FSMCallback)
                pass

            elif self.state == self.state_dict['INITIALIZING_LOCALIZATION']:
                if self.InitializeLocalization():
                    self.state = self.state_dict['INITIALIZING_PATH']
                    rospy.loginfo("[%s] Initialized intersection localization, initializing path." % (self.node_name))

            elif self.state == self.state_dict['INITIALIZING_PATH']:
                if self.InitializePath():
                    self.state = self.state_dict['TRAVERSING']
                    self.s = 0.0
                    self.traversing_start = rospy.Time.now()
                    rospy.loginfo("[%s] Initialized path, waiting for go signal." % (self.node_name))
                else:
                    rospy.loginfo("[%s] Could not initialize path. Trying again." % (self.node_name))

            elif self.state == self.state_dict['TRAVERSING']:
                '''open-loop'''
                if self.open_loop:
                    msg_cmds = Twist2DStamped()
                    msg_cmds.header.stamp = rospy.Time.now()

                    if 1.0 < (rospy.Time.now() - self.traversing_start).to_sec() and self.go:

                        pos, vel = self.pathPlanner.EvaluatePath(self.s)
                        dt = 0.01
                        _, vel2 = self.pathPlanner.EvaluatePath(self.s + dt)
                        self.alpha = self.v/np.linalg.norm(vel)

                        dir = vel/np.linalg.norm(vel)
                        dir2 = vel2/np.linalg.norm(vel2)
                        theta = np.arctan2(dir[1],dir[0])
                        theta2 = np.arctan2(dir2[1], dir2[0])
                        omega = (theta2 - theta)/dt

                        msg_cmds.v = self.v
                        msg_cmds.omega = self.alpha * omega

                        if (msg_cmds.v - 0.5 * math.fabs(msg_cmds.omega) * 0.1) < 0.065:
                            msg_cmds.v = 0.065 + 0.5 * math.fabs(msg_cmds.omega) * 0.1
                            self.alpha = self.alpha*msg_cmds.v/self.v

                        msg_cmds.v = msg_cmds.v * self.v_scale
                        msg_cmds.omega = msg_cmds.omega * self.omega_scale

                        self.s = self.s + self.alpha*(rospy.Time.now() - self.traversing_last_time).to_sec()

                        if (self.s > 0.99):
                            msg_cmds.v = self.v
                            msg_cmds.omega = 0.0

                            self.state = self.state_dict['DONE']
                            self.done_time = rospy.Time.now()

                    else:
                        msg_cmds.v = 0.0
                        msg_cmds.omega = 0.0

                    self.traversing_last_time = rospy.Time.now()
                    self.pub_cmds.publish(msg_cmds)

                else:
                    '''closed-loop'''
                    msg_lane_pose = LanePose()
                    msg_lane_pose.header.stamp = rospy.Time.now()

                    if 1.0 < (rospy.Time.now() - self.traversing_start).to_sec() and self.go:
                        pose, _ = self.poseEstimator.PredictState(msg_lane_pose.header.stamp)
                        dist, theta, curvature, self.s = self.pathPlanner.ComputeLaneError(pose, self.s)
                        #rospy.loginfo("the s is: "+str(self.s))

                        if (self.s > self.s_max):

                            msg_lane_pose.v_ref = self.v
                            msg_lane_pose.d = 0.0
                            msg_lane_pose.d_ref = 0.0
                            msg_lane_pose.phi = 0.0
                            msg_lane_pose.curvature_ref = 0.0
                            self.state = self.state_dict['DONE']
                            self.done_time = rospy.Time.now()

                        else:
                            msg_lane_pose.v_ref = self.v
                            msg_lane_pose.d = dist
                            msg_lane_pose.d_ref = 0.0
                            msg_lane_pose.phi = theta
                            msg_lane_pose.curvature_ref = curvature


                    else:
                        msg_lane_pose.v_ref = 0.0
                        msg_lane_pose.d = 0.0
                        msg_lane_pose.d_ref = 0.0
                        msg_lane_pose.phi = 0.0
                        msg_lane_pose.curvature_ref = 0.0

                    self.pub_lane_pose.publish(msg_lane_pose)

            elif self.state == self.state_dict['DONE']:

                # switch back to lane following if in lane
                if self.in_lane and (rospy.Time.now() - self.in_lane_time).to_sec() < self.in_lane_timeout:
                    msg_done = BoolStamped()
                    msg_done.header.stamp = rospy.Time.now()
                    msg_done.data = True
                    self.pub_done.publish(msg_done)
                    self.state = self.state_dict['IDLE']

                # go straight for 2.0 secs if not in lane yet. After 2 secs stop.
                else:
                    '''open-loop'''
                    if self.open_loop:
                        if (rospy.Time.now() - self.done_time).to_sec() < self.in_lane_wait_time:
                            msg_cmds = Twist2DStamped()
                            msg_cmds.header.stamp = rospy.Time.now()
                            msg_cmds.v = self.v * self.v_scale
                            msg_cmds.omega = 0.0 * self.omega_scale
                            self.pub_cmds.publish(msg_cmds)
                        else:
                            msg_cmds = Twist2DStamped()
                            msg_cmds.header.stamp = rospy.Time.now()
                            msg_cmds.v = 0.0 * self.v_scale
                            msg_cmds.omega = 0.0 * self.omega_scale
                            self.pub_cmds.publish(msg_cmds)
                            rospy.loginfo("[%s] Could not find lane. Stopping now." % (self.node_name))
                            self.state = self.state_dict['ERROR']

                    else:
                        '''closed-loop'''
                        if (rospy.Time.now() - self.done_time).to_sec() < self.in_lane_wait_time:
                            msg_lane_pose = LanePose()
                            msg_lane_pose.header.stamp = rospy.Time.now()
                            msg_lane_pose.v_ref = self.v
                            msg_lane_pose.d = 0.0
                            msg_lane_pose.d_ref = 0.0
                            msg_lane_pose.phi = 0.0
                            msg_lane_pose.curvature_ref = 0.0
                            self.pub_lane_pose.publish(msg_lane_pose)
                        else:
                            msg_lane_pose = LanePose()
                            msg_lane_pose.header.stamp = rospy.Time.now()
                            msg_lane_pose.v_ref = 0.0
                            msg_lane_pose.d = 0.0
                            msg_lane_pose.d_ref = 0.0
                            msg_lane_pose.phi = 0.0
                            msg_lane_pose.curvature_ref = 0.0
                            self.pub_lane_pose.publish(msg_lane_pose)
                            rospy.loginfo("[%s] Could not find lane. Stopping now." % (self.node_name))
                            self.state = self.state_dict['ERROR']

            else:
                rospy.loginfo("[%s] Something went wrong." % (self.node_name))

            rate.sleep()

    def InitializeLocalization(self):
        # waiting for april tag info (type intersection and which exit)
        if not self.active:
            return
        try:
            april_msg = rospy.wait_for_message('~apriltags_out', AprilTagsWithInfos, self.timeout)
        except rospy.ROSException:
            rospy.loginfo("[%s] Timeout waiting for april tag info." % (self.node_name))
            return False

        # find (valid) april tag
        best_w = 0
        best_idx = -1
        for idx, detection in enumerate(april_msg.detections):
            if april_msg.infos[idx].tag_type == self.tag_info.SIGN:
                if april_msg.infos[idx].traffic_sign_type in self.intersection_signs:
                    if detection.pose.pose.position.x < 0.8 and math.fabs(detection.pose.pose.orientation.w) > best_w:
                        best_w = math.fabs(detection.pose.pose.orientation.w)
                        best_idx = idx

        # return if no valid april tag was found
        if best_idx < 0:
            return False

        # initial position estimate
        x_init = self.nominal_start_positions[april_msg.infos[best_idx].traffic_sign_type][0]
        y_init = self.nominal_start_positions[april_msg.infos[best_idx].traffic_sign_type][1]
        theta_init = self.nominal_start_positions[april_msg.infos[best_idx].traffic_sign_type][2]

        if april_msg.infos[best_idx].traffic_sign_type in [self.tag_info.RIGHT_T_INTERSECT,
                                                              self.tag_info.LEFT_T_INTERSECT]:
            # Define range of initial positions: x,y: +- 2.5cm, theta: +- 5deg
            dx_init = np.linspace(-0.025, 0.025, 6)
            dy_init = np.linspace(-0.025, 0.025, 6)
            dtheta_init = np.linspace(-5.0 / 180.0 * np.pi, 5.0 / 180.0 * np.pi, 2)
        else:
            dx_init = np.linspace(-0.025, 0.025, 6)
            dy_init = np.linspace(-0.025, 0.025, 6)
            dtheta_init = np.linspace(-5.0 / 180.0 * np.pi, 5.0 / 180.0 * np.pi, 2)

        if april_msg.infos[best_idx].traffic_sign_type == self.tag_info.FOUR_WAY:
            self.intersectionLocalizer.SetEdgeModel('FOUR_WAY_INTERSECTION')
            self.intersection_type = 1
        else:
            self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')
            self.intersection_type = 0

        self.current_tag_info = april_msg.infos[best_idx].traffic_sign_type

        # waiting for camera image
        try:
            img_msg = rospy.wait_for_message("~img",
                                             CompressedImage, self.timeout)
        except rospy.ROSException:
            rospy.loginfo("[%s] Timeout waiting for camera image." % (self.node_name))
            return False

        # initialize intersection localizer
        img_processed, img_gray = self.intersectionLocalizer.ProcessRawImage(img_msg)

        best_likelihood = -1.0
        best_pose_meas = np.zeros(3, float)
        pose = np.zeros(3, float)

        for dx in dx_init:
            for dy in dy_init:
                for dtheta in dtheta_init:
                    pose[0] = x_init + dx
                    pose[1] = y_init + dy
                    pose[2] = theta_init + dtheta
                    valid_meas, pose_meas, likelihood = self.intersectionLocalizer.ComputePose(img_processed, pose)

                    if valid_meas and likelihood > best_likelihood:
                        best_likelihood = likelihood
                        best_pose_meas[:] = pose_meas

        if best_likelihood < 0.0:
            rospy.loginfo("[%s] Could not initialize intersection localizer." % (self.node_name))
            return False

        self.poseEstimator.Reset(best_pose_meas, img_msg.header.stamp)
        return True

    def InitializePath(self):
        # waiting for instructions where to go
         #if self.turn_type < 0 or (rospy.Time.now() - self.turn_type_time).to_sec() > self.turn_type_timeout:
        #     rospy.loginfo("[%s] No current turn type information." % (self.node_name))
        #     return False
        #TODO uncomment
        pose_init, _ = self.poseEstimator.PredictState(rospy.Time.now())

        rospy.loginfo("The duck will go: " +str(self.turn_type))
        pose_final = self.ComputeFinalPose(self.current_tag_info, self.turn_type)

        rospy.loginfo("[%s] Planning path from (%f,%f,%f) to (%f,%f,%f)." % (self.node_name, pose_init[0], pose_init[1],pose_init[2],pose_final[0],pose_final[1],pose_final[2]))

        if not self.pathPlanner.PlanPath(pose_init, pose_final):
            rospy.loginfo("[%s] Could not compute feasible path." % (self.node_name))
            return False

        else:
            msg_path_computed = BoolStamped()
            msg_path_computed.header.stamp = rospy.Time.now()
            msg_path_computed.data = True
            self.pub_path_computed.publish(msg_path_computed)
            msg_path_computed.data = False
            return True

    def InLaneCallback(self,msg):
        self.in_lane = msg.data
        self.in_lane_time = rospy.Time.now()


    def stopCar(self,event):
        msg_cmds = Twist2DStamped()
        msg_cmds.header.stamp = rospy.Time.now()
        msg_cmds.v = 0.0
        msg_cmds.omega = 0.0
        self.pub_cmds.publish(msg_cmds)

    def FSMCallback(self, msg):

        if self.state == self.state_dict['IDLE'] and msg.state == 'INTERSECTION_PLANNING':
            self.state = self.state_dict['INITIALIZING_LOCALIZATION']
            rospy.loginfo("[%s] Arrived at intersection, initializing intersection localization." % (self.node_name))

        if msg.state == 'INTERSECTION_CONTROL' and self.go == False:
            rospy.loginfo("[%s] Received go. Start traversing intersection." % (self.node_name))
            self.go = True

        if msg.state == 'ARRIVE_AT_STOP_LINE':
            self.SelfReset()

        if msg.state == 'NORMAL_JOYSTICK_CONTROL':
            self.state = self.state_dict['IDLE']
            self.SelfReset()

    def TurnTypeCallback(self, msg):
        self.turn_type = msg.data
        self.turn_type_time = rospy.Time.now()

    def ImageCallback(self, msg):
        if self.state == self.state_dict['TRAVERSING']:
            # predict pose
            pose_pred, _ = self.poseEstimator.PredictState(msg.header.stamp)
            msg_out = IntersectionPoseImg()
            msg_out.header.stamp = msg.header.stamp
            msg_out.x = pose_pred[0]
            msg_out.y = pose_pred[1]
            msg_out.theta = pose_pred[2]
            msg_out.type = self.intersection_type
            msg_out.img = msg
            self.pub_intersection_pose_img.publish(msg_out)

    def PoseCallback(self, msg):
        if msg.likelihood > 0.40:
            if math.fabs(msg.x) < 10.0 and math.fabs(msg.y) < 10.0:
                pose_meas = np.array([msg.x, msg.y, msg.theta])
                self.poseEstimator.UpdateWithPoseMeasurement(pose_meas, 1 * np.diag([0.1, 0.1, 1.0]),
                                                             msg.header.stamp)


    def CmdCallback(self, msg):
        if self.state == self.state_dict['INITIALIZING_PATH'] or self.state == self.state_dict['TRAVERSING']:
            cmd_msg = Twist2DStamped()
            cmd_msg.v = msg.v / self.v_scale
            cmd_msg.omega = msg.omega / self.omega_scale
            cmd_msg.header.stamp = msg.header.stamp
            self.poseEstimator.FeedCommandQueue(cmd_msg)

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))

if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('intersection_navigation_node', anonymous=False)

    # create the intersection navigation object
    node = IntersectionNavigation()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # run main logic
    node.MainLoop()
