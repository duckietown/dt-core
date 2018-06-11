#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import TurnIDandType, FSMState, BoolStamped, LanePose, Pose2DStamped, Twist2DStamped
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Point, PoseStamped, Pose, PointStamped
from nav_msgs.msg import Path
import time
import math
import json

class UnicornIntersectionNode(object):
    def __init__(self):
        self.node_name = "Unicorn Intersection Node"

        ## setup Parameters
        self.setupParams()

        ## Internal variables
        self.state = "JOYSTICK_CONTROL"
        self.active = False
        self.turn_type = -1


        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.cbTurnType)
        self.sub_fsm = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_int_go = rospy.Subscriber("~intersection_go", FSMState, self.cbIntersectionGo)
        #self.sub_debug = rospy.Subscriber("~debugging_start", Bool, self.cbDebugging)

        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.pub_LF_params = rospy.Publisher("~lane_filter_params", String, queue_size=1)


        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def changeLFParams(self, params, reset_time):
        data = {"params": params, "time": reset_time}
        msg = String()
        msg.data = json.dumps(data)
        self.pub_LF_params.publish(msg)

    def cbIntersectionGo(self, msg):
        if not msg.data: return

        while self.turn_type == -1:
            rospy.loginfo("Requested to start intersection, but we do not see an april tag yet.")
            rospy.sleep(2)

        if self.turn_type == 0:
            # TODO figure out magic
            params = {"param1": 11, "param2": 12}
            self.changeLFParams(params, self.time_left_turn)
            rospy.sleep(self.time_left_turn)
        if self.turn_type == 1:
            # TODO do not use yellow and replace red by white
            params = {"param1": 11, "param2": 12}
            self.changeLFParams(params, self.time_straight_turn)
            rospy.sleep(self.time_straight_turn)
        if self.turn_type == 2:
            #TODO in Lane control add feed forward
            rospy.sleep(self.time_right_turn)

        msg_done = BoolStamped()
        msg_done.data = True
        self.pub_int_done.publish(msg_done)

    def cbDebugging(self, msg):
        return

    def cbFSMState(self, msg):
        if self.state != msg.state and msg.state == "INTERSECTION_COORDINATION":
            self.turn_type = -1

        self.state = msg.state

    def cbTurnType(self, msg):
        self.turn_type = msg.data

    def setupParams(self):
        self.time_left_turn = self.setupParam("~time_left_turn", 2)
        self.time_straight_turn = self.setupParam("~time_straight_turn", 2)
        self.time_right_turn = self.setupParam("~time_right_turn", 2)


    def updateParams(self,event):
        self.time_left_turn = rospy.get_param("~time_left_turn")
        self.time_straight_turn = rospy.get_param("~time_straight_turn")
        self.time_right_turn = rospy.get_param("~time_right_turn")



    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[UnicornIntersectionNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('unicorn_intersection_node',anonymous=False)
    unicorn_intersection_node = UnicornIntersectionNode()
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
