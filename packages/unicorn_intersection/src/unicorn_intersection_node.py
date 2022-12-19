#!/usr/bin/env python3
import json
import numpy as np
import rospy
from duckietown_msgs.msg import BoolStamped, FSMState, LanePose, TurnIDandType, WheelEncoderStamped, Twist2DStamped
from std_msgs.msg import String

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import math 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform
# from duckietown_msgs.msg import WheelEncoderStamped
import message_filters
from tf2_ros import TransformBroadcaster
from tf import transformations as tr


class UnicornIntersectionNode(DTROS):
    def __init__(self, node_name):
        super(UnicornIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.node_name = node_name

        ## setup Parameters
        self.setupParams()

        ## Internal variables
        self.state = "JOYSTICK_CONTROL"
        self.active = False
        self.turn_type = -1
        self.tag_id = -1
        self.forward_pose = False

        ## Subscribers
        # self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.cbTurnType)
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.sub_fsm = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        # self.sub_int_go = rospy.Subscriber("~intersection_go", BoolStamped, self.cbIntersectionGo)
        # self.sub_lane_pose = rospy.Subscriber("~lane_pose_in", LanePose, self.cbLanePose)
        # self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        # self.pub_LF_params = rospy.Publisher("~lane_filter_params", String, queue_size=1)
        # self.pub_lane_pose = rospy.Publisher("~lane_pose_out", LanePose, queue_size=1)
        # self.pub_int_done_detailed = rospy.Publisher(
        #     "~intersection_done_detailed", TurnIDandType, queue_size=1
        # )
        self.sub_encoder_left = message_filters.Subscriber("~left_wheel_encoder_node/tick", WheelEncoderStamped)
        self.sub_encoder_right = message_filters.Subscriber("~right_wheel_encoder_node/tick", WheelEncoderStamped)

        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_left, self.sub_encoder_right], 1, 1
        )
        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        ## Deadreckoning 

        # introducing deadreckoning
        self.state = 0
        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None
        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0

        self.ticks_per_meter = 656.0
        self.debug = False
        self.wierd = True
        self.wheelbase = 0.108
        self.iter_ = 0

        # NOTE
        self.target_states = {-1: np.array([  [0.0,0.0], [0.0,0.0], [0.0,0.0]  ]),\
                              0: np.array([  [0.15,0.0], [0.3,0.0], [0.3,0.4]  ]),\
                              1: np.array([[0.15, 0.0], [0.3, 0.0], [0.4, 0.0]]),\
                              2: np.array([  [0.12,0.0], [0.12,-0.3], [0.12,-0.4]  ])
                              }
        self.len_states = len(self.target_states[self.turn_type])
        print(self.len_states)
        self.final_state = 0 

        #if path is created then only check for this 
        # IF 
        #self.callback(0,0)
        self.alpha = 0.0
        self.beta = 0.0

        self.log("Initialized controller")


    # def cbLanePose(self, msg):
    #     self.pub_lane_pose.publish(msg)

    # def changeLFParams(self, params, reset_time):
    #     data = {"params": params, "time": reset_time}
    #     msg = String()
    #     msg.data = json.dumps(data)
    #     self.pub_LF_params.publish(msg)

    def reset_odometry(self):
        self.state = 0
        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None
        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0

        self.ticks_per_meter = 656.0
        self.debug = False
        self.wierd = True
        self.wheelbase = 0.108
        self.iter_ = 0
        self.final_state = 0

    # def setwaypoint(self, waypoint):
    #     self.target_states = waypoint

    def cb_ts_encoders(self, left_encoder, right_encoder):
        timestamp_now = rospy.get_time()

        # Use the average of the two encoder times as the timestamp
        left_encoder_timestamp = left_encoder.header.stamp.to_sec()
        right_encoder_timestamp = right_encoder.header.stamp.to_sec()
        timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

        if not self.left_encoder_last:
            self.left_encoder_last = left_encoder
            self.right_encoder_last = right_encoder
            self.encoders_timestamp_last = timestamp
            self.encoders_timestamp_last_local = timestamp_now
            return

        # Skip this message if the time synchronizer gave us an older message
        dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
        dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
        if dtl.to_sec() < 0 or dtr.to_sec() < 0:
            self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        distance = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / self.wheelbase

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            #self.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
            dt = 1e-6
            #return

        self.tv = distance / dt
        self.rv = dyaw / dt

        if self.debug:
            self.loginfo(
                "Left wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (left_encoder.header.stamp.to_sec(), left_encoder.data, left_distance)
            )

            self.loginfo(
                "Right wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (right_encoder.header.stamp.to_sec(), right_encoder.data, right_distance)
            )

            self.loginfo(
                "TV = %.2f m/s\t RV = %.2f deg/s\t DT = %.4f" % (self.tv, self.rv * 180 / math.pi, dt)
            )

        dist = self.tv * dt
        dyaw = self.rv * dt

        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.q = tr.quaternion_from_euler(0, 0, self.yaw)
        self.timestamp = timestamp

        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

        if self.wierd and self.final_state == 0:
            print("iterator ",self.iter_)
            start_time = rospy.Time.now()

            self.state = 0

            car_control_msg = Twist2DStamped()
            car_control_msg.header.stamp = rospy.Time.now()
            car_control_msg.header.seq = 0

        # Add commands to car message
            #vpx = 0.5*(self.target_states[self.iter_][0]-self.x - self.alpha)
            #vpy = 0.06*(self.target_states[self.iter_][1]-self.y)
            car_control_msg.v = 0.2
            car_control_msg.omega = self.compute_omega(self.target_states[self.turn_type][self.iter_],self.x,self.y,self.yaw,dt)
            #= vpy/self.alpha

            print( " car commands  ",car_control_msg.omega)
            print("car control given",car_control_msg)

            if self.check_point( np.array([self.x,self.y]),self.target_states[self.turn_type][self.iter_] ):
                    self.iter_ += 1
                    car_control_msg.v = 0.0

                    if self.iter_ == len(self.target_states[self.turn_type]):
                        self.final_state = 1 # DONE
                        car_control_msg.v = 0
                        car_control_msg.omega = 0
                        self.car_cmd.publish(car_control_msg)
                        #self.reset_odometry()
                        return 

            print("target_state_ ",self.target_states[self.turn_type][self.iter_])
            self.car_cmd.publish(car_control_msg)
            print("cur state ",self.x,self.y,self.yaw)
            self.state = 1

    def cbIntersectionGo(self):
        rospy.loginfo("[%s] Recieved intersection go message from coordinator", self.node_name)

        while self.turn_type == -1:

            rospy.loginfo(
                "[%s] Requested to start intersection, but we do not see an april tag yet.", self.node_name
            )
            rospy.sleep(2)

        tag_id = self.tag_id
        turn_type = self.turn_type

        sleeptimes = [self.time_left_turn, self.time_straight_turn, self.time_right_turn]
        # LFparams = [self.LFparams_left, self.LFparams_straight, self.LFparams_right]
        # omega_ffs = [self.ff_left, self.ff_straight, self.ff_right]
        # omega_maxs = [self.omega_max_left, self.omega_max_straight, self.omega_max_right]
        # omega_mins = [self.omega_min_left, self.omega_min_straight, self.omega_min_right]

        # self.changeLFParams(LFparams[turn_type], sleeptimes[turn_type] + 1.0)
        # rospy.set_param("~lane_controller/omega_ff", omega_ffs[turn_type])
        # rospy.set_param("~lane_controller/omega_max", omega_maxs[turn_type])
        # rospy.set_param("~lane_controller/omega_min", omega_mins[turn_type])
        # Waiting for LF to adapt to new params
        # rospy.sleep(1)

        rospy.loginfo("Starting intersection control - driving to " + str(turn_type))
        self.ts_encoders.registerCallback(self.cb_ts_encoders)
        rospy.sleep(sleeptimes[turn_type])
        # rospy.set_param("~lane_controller/omega_ff", 0)
        # rospy.set_param("~lane_controller/omega_max", 999)
        # rospy.set_param("~lane_controller/omega_min", -999)

        # Publish intersection done
        msg_done = BoolStamped()
        msg_done.data = True
        self.pub_int_done.publish(msg_done)
        self.reset_odometry()

        # Publish intersection done detailed
        # msg_done_detailed = TurnIDandType()
        # msg_done_detailed.tag_id = tag_id
        # msg_done_detailed.turn_type = turn_type
        # self.pub_int_done_detailed.publish(msg_done_detailed)

    def cbFSMState(self, msg):
        if self.state != msg.state and msg.state == "NAVIGATE_INTERSECTION":
            self.turn_type = -1
            self.cbIntersectionGo()

        self.state = msg.state

    def cbTurnType(self, msg):
        self.tag_id = msg.tag_id
        if self.turn_type == -1:
            self.turn_type = msg.turn_type
        if self.debug_dir != -1:
            self.turn_type = self.debug_dir

    def setupParams(self):
        self.time_left_turn = self.setupParam("~time_left_turn", 2)
        self.time_straight_turn = self.setupParam("~time_straight_turn", 2)
        self.time_right_turn = self.setupParam("~time_right_turn", 2)
        # self.ff_left = self.setupParam("~ff_left", 1.5)
        # self.ff_straight = self.setupParam("~ff_straight", 0)
        # self.ff_right = self.setupParam("~ff_right", -1)
        # self.LFparams_left = self.setupParam("~LFparams_left", 0)
        # self.LFparams_straight = self.setupParam("~LFparams_straight", 0)
        # self.LFparams_right = self.setupParam("~LFparams_right", 0)
        # self.omega_max_left = self.setupParam("~omega_max_left", 999)
        # self.omega_max_straight = self.setupParam("~omega_max_straight", 999)
        # self.omega_max_right = self.setupParam("~omega_max_right", 999)
        # self.omega_min_left = self.setupParam("~omega_min_left", -999)
        # self.omega_min_straight = self.setupParam("~omega_min_straight", -999)
        # self.omega_min_right = self.setupParam("~omega_min_right", -999)

        self.debug_dir = self.setupParam("~debug_dir", -1)

    def updateParams(self, event):
        self.time_left_turn = rospy.get_param("~time_left_turn")
        self.time_straight_turn = rospy.get_param("~time_straight_turn")
        self.time_right_turn = rospy.get_param("~time_right_turn")
        # self.ff_left = rospy.get_param("~ff_left")
        # self.ff_straight = rospy.get_param("~ff_straight")
        # self.ff_right = rospy.get_param("~ff_right")
        # self.LFparams_left = rospy.get_param("~LFparams_left")
        # self.LFparams_straight = rospy.get_param("~LFparams_straight")
        # self.LFparams_right = rospy.get_param("~LFparams_right")
        # self.omega_max_left = rospy.get_param("~omega_max_left")
        # self.omega_max_straight = rospy.get_param("~omega_max_straight")
        # self.omega_max_right = rospy.get_param("~omega_max_right")
        # self.omega_min_left = rospy.get_param("~omega_min_left")
        # self.omega_min_straight = rospy.get_param("~omega_min_straight")
        # self.omega_min_right = rospy.get_param("~omega_min_right")

        self.debug_dir = rospy.get_param("~debug_dir")

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo(f"[{self.node_name}] {param_name} = {value} ")
        return value

    def onShutdown(self):
        rospy.loginfo("[UnicornIntersectionNode] Shutdown.")

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * math.pi:
            return theta - 2 * math.pi
        elif theta < -2 * math.pi:
            return theta + 2 * math.pi
        else:
            return theta      

    def path_plan(self,obstacle,lane):
            return 0

    def compute_omega(self,targetxy,x,y,current,dt):
        factor = 1 # PARAM 

        print("compute omega targetxy", targetxy)
        print("compute omega current",x,y)

        target_yaw = np.arctan2( (targetxy[1] - y),(targetxy[0]- x) )

        print("target yaw ", np.rad2deg(target_yaw))
        print("currnt_yaw ", np.rad2deg(current))

        omega = factor* ((target_yaw - current))

        return omega

    def check_point(self,current_point,target_point):
        threshold = 0.1
        threshold_x = 0.08
        dist_x = np.zeros((1,2))
        dist_x[0,0] = (current_point[0]-self.alpha) - target_point[0]
        dist_x[0,1] = (current_point[1]) - target_point[1]
        if self.iter_ == (self.len_states - 1):
            if abs(dist_x[0,1]) < threshold_x:
                return True

            return False

        else:
            dist = np.sqrt(((current_point[0]-self.alpha) - target_point[0])**2 + ((current_point[1]-self.alpha) - target_point[1])**2 )
            print("check if we reached ")
            print("dist ",dist)
            print(" current and target point",current_point,target_point)
            print("-"*10)

            if (abs(dist_x[0,0])) > threshold_x or (dist) < threshold:
                return True

            return False

if __name__ == "__main__":
    # rospy.init_node("unicorn_intersection_node", anonymous=False)
    unicorn_intersection_node = UnicornIntersectionNode(node_name="unicorn_intersection_node")
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
