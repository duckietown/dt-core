#!/usr/bin/env python
from __future__ import print_function
from random import random
import rospy
from duckietown_msgs.msg import CoordinationClearanceETHZ17, FSMState, BoolStampedETHZ17, Twist2DStampedETHZ17
from duckietown_msgs.msg import SignalsDetectionETHZ17, CoordinationSignalETHZ17
from std_msgs.msg import String
from time import time

UNKNOWN = 'UNKNOWN'


class State:
    LANE_FOLLOWING          = 'LANE_FOLLOWING'
    AT_STOP_CLEARING        = 'AT_STOP_CLEARING'
    AT_STOP_CLEAR           = 'AT_STOP_CLEAR'
    RESERVING               = 'RESERVING'
    CONFLICT                = 'CONFLICT'
    GO                      = 'GO'
    TL_SENSING              = 'TL_SENSING'
    INTERSECTION_NAVIGATION = 'INTERSECTION_NAVIGATION'


class VehicleCoordinator():
    # Parameters
    T_MAX_RANDOM = 0.0 + 4.0  # seconds
    T_CROSS      = 0.0 + 6.0  # seconds
    T_SENSE      = 0.0 + 2.0  # seconds

    # We communicate that the coordination mode has started
    def __init__(self):
        rospy.loginfo('The Coordination Mode has Started')

        # Determine the state of the bot
        #self.state = State.AT_STOP_CLEARING
	self.state = State.LANE_FOLLOWING
        self.last_state_transition = time()
        self.random_delay = 0

        # Parameters
        self.intersection_go_published = True


        self.node = rospy.init_node('veh_coordinator', anonymous=True)

        # Are we in a situation with traffic lights? Initially always false.
        if rospy.get_param("~intersectionType") == "trafficLight":
            self.traffic_light_intersection = True
        else:
            self.traffic_light_intersection = False
	self.traffic_light_intersection = False

        rospy.loginfo('[simple_coordination_node]: trafficLight=%s' % str(self.traffic_light_intersection))

        # Subscriptions
        self.mode = 'LANE_FOLLOWING'
        rospy.Subscriber('~mode', FSMState, lambda msg: self.set('mode', msg.state))

        # Initialize variables
        self.traffic_light = UNKNOWN
        self.veh_detected  = UNKNOWN
        # self.right_veh = UNKNOWN
        # self.opposite_veh = UNKNOWN


	self.veh_detected = UNKNOWN
	self.detected_car = UNKNOWN
	rospy.Subscriber('~signals_detection', SignalsDetectionETHZ17, self.process_signals_detection) # see below

        # Publishing
        self.clearance_to_go = CoordinationClearanceETHZ17.NA
        # state of the clearance
        self.clearance_to_go_pub = rospy.Publisher('~clearance_to_go', CoordinationClearanceETHZ17, queue_size=10)
        # signal for the intersection
        self.pub_intersection_go = rospy.Publisher('~intersection_go', BoolStampedETHZ17, queue_size=1)
        self.pub_coord_cmd       = rospy.Publisher('~car_cmd', Twist2DStampedETHZ17, queue_size=1)

        # set the light to be off
        self.roof_light = CoordinationSignalETHZ17.OFF
        # publish that
        self.roof_light_pub         = rospy.Publisher('~change_color_pattern', String, queue_size=10)
        self.coordination_state_pub = rospy.Publisher('~coordination_state', String, queue_size=10)

        while not rospy.is_shutdown():
            self.loop()
            rospy.sleep(0.1)

    def set_state(self, state):
        self.state = state
        self.last_state_transition = time()

        if self.state == State.AT_STOP_CLEARING:
            self.reset_signals_detection()
            # self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
            # after setting everything to unknown, we turn on the light
            self.roof_light = CoordinationSignalETHZ17.ON
        elif self.state == State.AT_STOP_CLEAR:
            self.roof_light = CoordinationSignalETHZ17.ON
            #self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
        # elif self.state == State.RESERVING:
            #self.roof_light = CoordinationSignalETHZ17.SIGNAL_B
        elif self.state == State.CONFLICT:
            #self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
            self.roof_light = CoordinationSignalETHZ17.OFF
        elif self.state == State.GO and not self.traffic_light_intersection:
            self.roof_light = CoordinationSignalETHZ17.ON
        #else:
            #self.roof_light = CoordinationSignalETHZ17.OFF

        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearanceETHZ17.GO
        else:
            self.clearance_to_go = CoordinationClearanceETHZ17.WAIT

        rospy.logdebug('[coordination_node_ETHZ17] Transitioned to state' + self.state)

    # Define the time at this current state
    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):
        self.__dict__[name] = value

    # Definition of each signal detection
    def process_signals_detection(self, msg):
        # self.set('traffic_light', msg.traffic_light_state)
        # self.set('right_veh', msg.right)
        # self.set('opposite_veh', msg.front)
        self.set('veh_detected', msg.led_detected)
        self.set('veh_not_detected', msg.no_led_detected)

    # definition which resets everything we know
    def reset_signals_detection(self):
        self.traffic_light = UNKNOWN
        # self.right_veh    = UNKNOWN
        # self.opposite_veh = UNKNOWN
        self.veh_detected     = UNKNOWN
        self.veh_not_detected = UNKNOWN

    # publishing the topics
    def publish_topics(self):
        now = rospy.Time.now()
        self.clearance_to_go_pub.publish(CoordinationClearanceETHZ17(status=self.clearance_to_go))
        # Publish intersection_go flag
        if self.clearance_to_go == CoordinationClearanceETHZ17.GO:
            msg = BoolStampedETHZ17()
            msg.header.stamp = now
            msg.data = True
            self.pub_intersection_go.publish(msg)
            # TODO: publish intersection go only once.
        # Publish LED
        self.roof_light_pub.publish(self.roof_light)
        # Publish car state
        car_cmd_msg = Twist2DStampedETHZ17(v=0.0, omega=0.0)
        car_cmd_msg.header.stamp = now
        self.pub_coord_cmd.publish(car_cmd_msg)
        # Publish coordination state
        self.coordination_state_pub.publish(data=self.state)

    # definition of the loop
    def loop(self):
        self.reconsider()
        self.publish_topics()

    def reconsider(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                self.reset_signals_detection()
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    # our standard case: setting at stop clearing!
                    self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.AT_STOP_CLEARING:
            # if self.right_veh != SignalsDetectionETHZ17.NO_CAR or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C:
            if self.veh_detected == SignalsDetectionETHZ17.CARS:  # if we are seeing other cars (i.e. we cannot go)
                self.roof_light = CoordinationSignalETHZ17.OFF
                self.random_delay = random() * self.T_MAX_RANDOM
                print("Other vehicle are waiting as well. Will wait for %.2f s" % self.random_delay)
                self.set_state(State.CONFLICT)
            elif self.time_at_current_state() > self.T_CROSS + self.T_SENSE:
               	# TODO: What can't we go here if no vehicles are detected?
		self.set_state(State.AT_STOP_CLEAR)

        elif self.state == State.AT_STOP_CLEAR:
            # if self.right_veh != SignalsDetectionETHZ17.NO_CAR or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C:
            if self.veh_detected == SignalsDetectionETHZ17.CARS:  # if we are seeing other cars (i.e. we cannot go)
                self.set_state(State.AT_STOP_CLEARING)
            else:
                self.set_state(State.GO)

        # elif self.state == State.RESERVING:
        #  if self.right_veh != SignalsDetectionETHZ17.NO_CAR:
        #     self.set_state(State.AT_STOP_CLEARING)
        # elif self.time_at_current_state() > 2 * self.T_SENSE:
        #    if self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B:
        #       self.random_delay = random() * self.T_MAX_RANDOM
        #      print ("Other vehicle reserving as well. Will wait for %.2f s" % self.random_delay)
        #     self.set_state(State.CONFLICT)
        # else:
        #    self.set_state(State.GO)

        elif self.state == State.GO:
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)

        elif self.state == State.CONFLICT:
            # if self.right_veh != SignalsDetectionETHZ17.NO_CAR or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C:
            # self.set_state(State.AT_STOP_CLEARING)
            if self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEAR)

        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetectionETHZ17.GO:
                self.set_state(State.GO)


if __name__ == '__main__':
    car = VehicleCoordinator()
    rospy.spin()
