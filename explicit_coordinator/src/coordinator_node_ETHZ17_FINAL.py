#!/usr/bin/env python
from __future__ import print_function
from random import random
import rospy
from duckietown_msgs.msg import CoordinationClearance, FSMState, BoolStamped, Twist2DStamped, AprilTagsWithInfos
from duckietown_msgs.msg import SignalsDetection, CoordinationSignal
from std_msgs.msg import String
from time import time

UNKNOWN = 'UNKNOWN'

class State:
    LANE_FOLLOWING = 'LANE_FOLLOWING'
    AT_STOP_CLEARING = 'AT_STOP_CLEARING'
    SACRIFICE = 'SACRIFICE'
    SOLVING_UNKNOWN = 'SOLVING_UNKNOWN'
    GO = 'GO'
    KEEP_CALM = 'KEEP_CALM'	
    TL_SENSING = 'TL_SENSING'
    INTERSECTION_NAVIGATION = 'INTERSECTION_NAVIGATION'

class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    T_MAX_RANDOM = 5.0 # seconds
    T_CROSS = 6.0      # seconds
    T_SENSE = 2.0      # seconds
    T_UNKNOWN = 1.0    # seconds
    T_MIN_RANDOM = 2.0 # seconds
    T_KEEP_CALM = 4.0  # seconds


    def __init__(self):
        # We communicate that the coordination mode has started
        rospy.loginfo('The Coordination Mode has Started')

        # Determine the state of the bot
        self.state = State.LANE_FOLLOWING
        self.last_state_transition = time()
        self.random_delay = 0

        # Node name
        self.node_name = rospy.get_name()

        # Initialize flag
        self.intersection_go_published = False

        self.node = rospy.init_node('veh_coordinator', anonymous=True)

        # Parameters
        self.traffic_light_intersection = UNKNOWN

        # Initialize detection
        self.traffic_light = UNKNOWN
        self.right_veh = UNKNOWN
        self.opposite_veh = UNKNOWN

        # Initialize mode
        self.mode = 'LANE_FOLLOWING'

        # Subscriptions
        rospy.Subscriber('~mode', FSMState, lambda msg: self.set('mode', msg.state))
        rospy.Subscriber('~apriltags', AprilTagsWithInfos, self.set_traffic_light)
        rospy.Subscriber('~signals_detection', SignalsDetection, self.process_signals_detection)

        # Initialize clearance to go
        self.clearance_to_go = CoordinationClearance.NA

        # Set the light to be off
        self.roof_light = CoordinationSignal.OFF

        # Publishing
        self.clearance_to_go_pub    = rospy.Publisher('~clearance_to_go', CoordinationClearance, queue_size=10)
        self.pub_intersection_go    = rospy.Publisher('~intersection_go', BoolStamped, queue_size=1)
        self.pub_coord_cmd          = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.roof_light_pub         = rospy.Publisher('~change_color_pattern', String, queue_size=10)
        self.coordination_state_pub = rospy.Publisher('~coordination_state', String, queue_size=10)

        while not rospy.is_shutdown():
            self.loop()
            rospy.sleep(0.1)

    def set_traffic_light(self,msg):
        for item in msg.infos:
            if item.traffic_sign_type == 17:
                    self.traffic_light_intersection = True
                    break
            else:
                    self.traffic_light_intersection = False

        if self.traffic_light_intersection != UNKNOWN:
            # Print
            if self.traffic_light_intersection:
                rospy.loginfo('[%s] Intersection with traffic light' %(self.node_name))
            else:
                rospy.loginfo('[%s] Intersection without traffic light' %(self.node_name))

    def set_state(self, state):
        # Update only when changing state
        if self.state != state:
            rospy.loginfo('[%s] Transitioned from %s to %s' %(self.node_name,self.state,state))
            self.last_state_transition = time()
            self.state = state

        # Set roof light
        if self.state == State.AT_STOP_CLEARING:
            # self.reset_signals_detection()
           self.roof_light = CoordinationSignal.SIGNAL_A
        elif self.state == State.SACRIFICE:
            self.roof_light = CoordinationSignal.OFF
        elif self.state == State.KEEP_CALM:
            self.roof_light = CoordinationSignal.SIGNAL_A
        elif self.state == State.GO and not self.traffic_light_intersection:
            self.roof_light = CoordinationSignal.SIGNAL_GREEN
        elif self.state == State.LANE_FOLLOWING or self.state == State.TL_SENSING:
            self.roof_light = CoordinationSignal.OFF

        rospy.logdebug('[coordination_node] Transitioned to state' + self.state)

    # Define the time at this current state
    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):
        self.__dict__[name] = value

        # Initialization of the state and of the type of intersection
        if name == 'mode':
            if value == 'JOYSTICK_CONTROL' or value == 'COORDINATION':
                self.set_state(State.LANE_FOLLOWING)
                self.traffic_light_intersection = UNKNOWN

    # Definition of each signal detection
    def process_signals_detection(self, msg):
        self.set('traffic_light', msg.traffic_light_state)
        self.set('right_veh', msg.right)
        self.set('opposite_veh', msg.front)

    # definition which resets everything we know
    def reset_signals_detection(self):
        self.traffic_light = UNKNOWN
        self.right_veh     = UNKNOWN
        self.opposite_veh  = UNKNOWN

    # publishing the topics
    def publish_topics(self):
        now = rospy.Time.now()
        self.clearance_to_go_pub.publish(CoordinationClearance(status=self.clearance_to_go))

        # Publish intersection_go flag
        if self.clearance_to_go == CoordinationClearance.GO and not self.intersection_go_published:
            msg = BoolStamped()
            msg.header.stamp = now
            msg.data = True
            self.pub_intersection_go.publish(msg)
            self.intersection_go_published = True
            rospy.loginfo('[%s] Go!' %(self.node_name))

        # Publish LEDs
        self.roof_light_pub.publish(self.roof_light)

        car_cmd_msg = Twist2DStamped(v=0.0,omega=0.0)
        car_cmd_msg.header.stamp = now
        self.pub_coord_cmd.publish(car_cmd_msg)
        self.coordination_state_pub.publish(data=self.state)

    # definition of the loop
    def loop(self):
        if self.traffic_light_intersection != UNKNOWN:
            self.reconsider()
        self.publish_topics()

    def reconsider(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                # Reset detections
                self.reset_signals_detection()

                # Go to state (depending whether there is a traffic light)
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.AT_STOP_CLEARING:
            # First measurement not seen yet
            if self.right_veh == UNKNOWN or self.opposite_veh == UNKNOWN:
                self.random_delay = 1 + random()*self.T_UNKNOWN
                self.set_state(State.SOLVING_UNKNOWN)
            # Other cars  detected
            elif self.right_veh == SignalsDetection.SIGNAL_A or self.opposite_veh == SignalsDetection.SIGNAL_A:
                self.random_delay = self.T_MIN_RANDOM + random()*(self.T_MAX_RANDOM-self.T_MIN_RANDOM)
                self.set_state(State.SACRIFICE)
                rospy.loginfo("[%s] Other vehicle are waiting as well. Will wait for %.2f s" %(self.node_name,self.random_delay))
            # No cars detected
            else:
                self.set_state(State.KEEP_CALM)

        elif self.state == State.GO:
            self.clearance_to_go = CoordinationClearance.GO
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)

        elif self.state == State.SACRIFICE:
            # Wait a random delay
            if self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.KEEP_CALM:
            # Other cars  detected
            if self.right_veh == SignalsDetection.SIGNAL_A or self.opposite_veh == SignalsDetection.SIGNAL_A:
                self.set_state(State.SACRIFICE)
            # No cars  detected
            else:
                if self.time_at_current_state() > self.T_KEEP_CALM:
                    self.set_state(State.GO)

        elif self.state == State.SOLVING_UNKNOWN:
            if self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetection.GO:
                self.set_state(State.GO)

        # If not GO, pusblish wait
        if self.state != State.GO:
            # Initialize intersection_go_published
            self.intersection_go_published = False
            # Publish wait
            self.clearance_to_go = CoordinationClearance.WAIT

if __name__ == '__main__':
    car = VehicleCoordinator()
    rospy.spin()
