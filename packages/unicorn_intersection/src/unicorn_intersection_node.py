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
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.sub_fsm = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
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
        self.final_state = 0 

        self.alpha = 0.0
        self.beta = 0.0

        self.log("Initialized controller")

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
            dt = 1e-6

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
            car_control_msg.v = 0.2
            car_control_msg.omega = self.compute_omega(self.target_states[self.turn_type][self.iter_],self.x,self.y,self.yaw,dt)

            if self.check_point( np.array([self.x,self.y]),self.target_states[self.turn_type][self.iter_] ):
                    self.iter_ += 1
                    car_control_msg.v = 0.0

                    if self.iter_ == len(self.target_states[self.turn_type]):
                        self.final_state = 1 # DONE
                        car_control_msg.v = 0
                        car_control_msg.omega = 0
                        self.car_cmd.publish(car_control_msg)
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

        rospy.loginfo("Starting intersection control - driving to " + str(turn_type))
        self.ts_encoders.registerCallback(self.cb_ts_encoders)
        rospy.sleep(sleeptimes[turn_type])

        # Publish intersection done
        msg_done = BoolStamped()
        msg_done.data = True
        self.pub_int_done.publish(msg_done)
        self.reset_odometry()


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
        self.debug_dir = self.setupParam("~debug_dir", -1)

    def updateParams(self, event):
        self.time_left_turn = rospy.get_param("~time_left_turn")
        self.time_straight_turn = rospy.get_param("~time_straight_turn")
        self.time_right_turn = rospy.get_param("~time_right_turn")
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
        target_yaw = np.arctan2( (targetxy[1] - y),(targetxy[0]- x) )
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
    unicorn_intersection_node = UnicornIntersectionNode(node_name="unicorn_intersection_node")
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
