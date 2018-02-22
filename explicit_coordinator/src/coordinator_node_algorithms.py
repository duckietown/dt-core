#!/usr/bin/env python
from __future__ import print_function
from random import random
from random import randint
import rospy
from duckietown_msgs.msg import CoordinationClearance, FSMState, BoolStamped, Twist2DStamped
from duckietown_msgs.msg import SignalsDetection, CoordinationSignal
from std_msgs.msg import String
from time import time


UNKNOWN = 'UNKNOWN'

class State:
    #approach 0(last year's code)
    LANE_FOLLOWING = 'LANE_FOLLOWING'
    AT_STOP_CLEARING = 'AT_STOP_CLEARING'
    AT_STOP_CLEAR = 'AT_STOP_CLEAR'
    RESERVING = 'RESERVING'
    CONFLICT = 'CONFLICT'
    GO = 'GO'
    TL_SENSING = 'TL_SENSING'
    INTERSECTION_NAVIGATION = 'INTERSECTION_NAVIGATION'
    
    #approach 1
    #LANE_FOLLOWING, GO, TL_SENSING, AT_STOP_CLEARING
    WAIT_FOR_RIGHT = 'WAIT_FOR_RIGHT'
    WAIT_FOR_OPPOSITE = 'WAIT_FOR_OPPOSITE'


    #approach 2
    #LANE_FOLLOWING, GO, TL_SENSING
    NO_COLOR_QUEUE = 'NO_COLOR_QUEUE'
    RED_QUEUE = 'RED_QUEUE'
    NEGOTIATION_QUEUE = 'NEGOTIATION_QUEUE'

    #approach 3
    #LANE_FOLLOWING, GO, TL_SENSING
    CHECKING = 'CHECKING'
    WAIT = 'WAIT'


class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    T_MAX_RANDOM = 2.0 # seconds
    T_CROSS = 6.0  # seconds
    T_SENSE = 2.0      # seconds

    def __init__(self):
        rospy.loginfo('Coordination Mode Started')

        self.state = State.LANE_FOLLOWING
        self.last_state_transition = time()
        self.random_delay = 0

        self.intersection_go_published = False

        self.node = rospy.init_node('veh_coordinator', anonymous=True)

        # Parameters

        if rospy.get_param("~intersectionType") == "trafficLight":
            self.traffic_light_intersection = True
        else:
            self.traffic_light_intersection = False

        rospy.loginfo('[simple_coordination_node]: trafficLight=%s' % str(self.traffic_light_intersection))


        # Subscriptions
        self.mode = 'LANE_FOLLOWING'
        rospy.Subscriber('~mode', FSMState, lambda msg: self.set('mode', msg.state))
        #int mode: {LANE_FOLLOWING, COORDINATION, INTERSECTION_NAVIGATION}

        self.traffic_light = UNKNOWN
        self.right_veh = UNKNOWN
        self.opposite_veh = UNKNOWN
        self.left_veh = UNKNOWN
        rospy.Subscriber('~signals_detection', SignalsDetectionETHZ17, self.process_signals_detection)

        #int detection: {NO_CAR, SIGNAL_A=11, SIGNAL_B=12, SIGNAL_C=13}


        # Publishing
        
        #coordination_clearance: int status {NA, WAIT, GO}
        self.clearance_to_go = CoordinationClearanceETHZ17.NA
        self.clearance_to_go_pub = rospy.Publisher('~clearance_to_go', CoordinationClearanceETHZ17, queue_size=10)
        self.pub_intersection_go = rospy.Publisher('~intersection_go', BoolStampedETHZ17, queue_size=1)
        self.pub_coord_cmd = rospy.Publisher('~car_cmd', Twist2DStampedETHZ17, queue_size=1)

        #???
        self.roof_light = CoordinationSignalETHZ17.OFF

        self.roof_light_pub = rospy.Publisher('~change_color_pattern', String, queue_size=10)


        self.coordination_state_pub = rospy.Publisher('~coordination_state', String, queue_size=10)

        while not rospy.is_shutdown():
            self.loop()
            rospy.sleep(0.1)


#vvvvvvvvvvvv different set_state methods for different approaches
#adjust reconsider methods below accordingly

    #approach 0(last year's code)
    def set_state0(self, state):
        if self.state != state:
            self.last_state_transition = time()
        self.state = state

        if self.state == State.AT_STOP_CLEARING:
            self.reset_signals_detection()
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
        elif self.state == State.AT_STOP_CLEAR:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
        elif self.state == State.RESERVING:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_B
        elif self.state == State.CONFLICT:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
        elif self.state == State.GO and not self.traffic_light_intersection:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_C
        else:
            self.roof_light = CoordinationSignalETHZ17.OFF

        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearanceETHZ17.GO
            rospy.loginfo("I GOOOOOOOO ")
        else:
            self.clearance_to_go = CoordinationClearanceETHZ17.WAIT

        rospy.loginfo("I AM CHANGING STATE TO: [%s]" %(state))
        rospy.logdebug('[simple_coordination_node] Transitioned to state' + self.state)


    #approach 1
    def set_state1(self, state):
        self.state = state
        self.last_state_transition = time()

        if self.state == State.AT_STOP_CLEARING:
            self.reset_signals_detection()
            self.roof_light = CoordinationSignalETHZ17.OFF
        elif (self.state == State.WAIT_FOR_RIGHT or
            self.state == State.WAIT_FOR_OPPOSITE):
            self.roof_light = CoordinationSignalETHZ17.OFF
        else:
            self.roof_light = CoordinationSignalETHZ17.OFF
        
        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearanceETHZ17.GO
            rospy.loginfo("I GOOOOOOOO ")
        else:
            self.clearance_to_go = CoordinationClearanceETHZ17.WAIT

        rospy.loginfo("I AM CHANGING STATE TO: [%s]" %(state))
        rospy.logdebug('[simple_coordination_node] Transitioned to state' + self.state)


    #approach 2
    def set_state(self, state):
        self.state = state
        self.last_state_transition = time()
        if self.state == State.NO_COLOR_QUEUE:
            self.roof_light = CoordinationSignalETHZ17.OFF
        elif self.state == State.RED_QUEUE:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_C
        elif self.state == State.NEGOTIATION_QUEUE:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_B
        else:
            self.roof_light = CoordinationSignalETHZ17.OFF
        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearanceETHZ17.GO
            rospy.loginfo("I GOOOOOOOO ")
        else:
            self.clearance_to_go = CoordinationClearanceETHZ17.WAIT

        rospy.loginfo("I AM CHANGING STATE TO: [%s]" %(state))
        rospy.logdebug('[simple_coordination_node] Transitioned to state' + self.state)


    #approach 3(exponential backoff)
    def set_state3(self, state):
        if self.state != state:
            self.last_state_transition = time()
        self.state = state

        if self.state == State.CHECKING:
<<<<<<< HEAD
            self.roof_light = CoordinationSignal.SIGNAL_A
        elif self.state == State.WAIT:
            self.roof_light = CoordinationSignal.SIGNAL_B
        else:
            self.roof_light = CoordinationSignal.OFF

        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearance.GO
            rospy.loginfo("I GOOOOOOOO ")
        else:
            self.clearance_to_go = CoordinationClearance.WAIT
=======
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
        elif self.state == State.WAIT:
            self.roof_light = CoordinationSignalETHZ17.SIGNAL_B
        else:
            self.roof_light = CoordinationSignalETHZ17.OFF

        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearanceETHZ17.GO
            rospy.loginfo("I GOOOOOOOO ")
        else:
            self.clearance_to_go = CoordinationClearanceETHZ17.WAIT
>>>>>>> devel-explicit-coord-gzardini

        rospy.loginfo("I AM CHANGING STATE TO: [%s]" %(state))
        rospy.logdebug('[simple_coordination_node] Transitioned to state' + self.state)


#^^^^^^^^^^^^^^ end of set_state methods ^^^^^^^^^^^^^^^^^^^^^ 

    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):
        self.__dict__[name] = value

    def process_signals_detection(self, msg):
        self.set('traffic_light', msg.traffic_light_state)
        self.set('right_veh', msg.right)
        self.set('opposite_veh', msg.front)
        self.set('left_veh', msg.left)

    def reset_signals_detection(self):
        self.traffic_light = UNKNOWN
        self.right_veh = UNKNOWN
        self.opposite_veh = UNKNOWN
        self.left_veh = UNKNOWN

    def publish_topics(self):
        now = rospy.Time.now()
        self.clearance_to_go_pub.publish(CoordinationClearanceETHZ17(status=self.clearance_to_go))
        # Publish intersection_go flag
        if (self.clearance_to_go == CoordinationClearanceETHZ17.GO):
            msg = BoolStampedETHZ17()
            msg.header.stamp = now
            msg.data = True
            self.pub_intersection_go.publish(msg)
            # TODO: publish intersection go only once.
        self.roof_light_pub.publish(self.roof_light)

        car_cmd_msg = Twist2DStampedETHZ17(v=0.0,omega=0.0)
        car_cmd_msg.header.stamp = now
        self.pub_coord_cmd.publish(car_cmd_msg)
        self.coordination_state_pub.publish(data=self.state)

    def loop(self):
        self.reconsider()
        self.publish_topics()



#vvvvvvvvv different reconsider methods for different approaches vvvvvvvvvvvvvvv

    #approach 0(last year's code)
    def reconsider0(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                self.reset_signals_detection()
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    self.set_state(State.AT_STOP_CLEARING)
        elif self.state == State.AT_STOP_CLEARING:
            if (self.right_veh != SignalsDetectionETHZ17.NO_CARS or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C):
                self.set_state(State.AT_STOP_CLEARING)
                #rospy.loginfo("entered clearing")
                rospy.loginfo("time at current step is [%s]" % str(self.time_at_current_state()))

            elif self.time_at_current_state() > self.T_CROSS + self.T_SENSE:
                self.set_state(State.AT_STOP_CLEAR)
                #rospy.loginfo("entered other branch")

        elif self.state == State.AT_STOP_CLEAR:
            if (self.right_veh != SignalsDetectionETHZ17.NO_CAR or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C):
                self.set_state(State.AT_STOP_CLEARING)
            else:
                self.set_state(State.RESERVING)
        elif self.state == State.RESERVING:
            if self.right_veh != SignalsDetectionETHZ17.NO_CAR:
                self.set_state(State.AT_STOP_CLEARING)
            elif self.time_at_current_state() > 2 * self.T_SENSE:
                if self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B:
                    self.random_delay = random() * self.T_MAX_RANDOM
                    print ("Other vehicle reserving as well. Will wait for %.2f s" % self.random_delay)
                    self.set_state(State.CONFLICT)
                else:
                    self.set_state(State.GO)
        elif self.state == State.GO:
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)
        elif self.state == State.CONFLICT:
            if (self.right_veh != SignalsDetectionETHZ17.NO_CAR or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C):
                self.set_state(State.AT_STOP_CLEARING)
            elif self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEAR)
        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetectionETHZ17.GO:
                self.set_state(State.GO)



    #approach 1
    #SIGNAL_A = RIGHT_OCCUPIED, SIGNAL_B = ABOUT TO GO or GOING
    def reconsider1(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                self.reset_signals_detection()
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    self.set_state(State.AT_STOP_CLEARING)
        elif self.state == AT_STOP_CLEARING:
            if (self.right_veh == SignalsDetectionETHZ17.NO_CAR and
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_A):
                self.roof_light = CoordinationSignalETHZ17.SIGNAL_B
                self.set_state(State.GO)
            elif ((self.right_veh == SignalsDetectionETHZ17.SIGNAL_A or
                self.right_veh == SignalsDetectionETHZ17.SIGNAL_B) and
                (self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_A) or
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B):
                if(randint(1,4) == 4):
                    self.set_state(State.WAIT_FOR_RIGHT)
                else:
                    self.set_state(State.AT_STOP_CLEARING)
            elif (self.right_veh == SignalsDetectionETHZ17.NO_CAR and
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B):
                if(randint(1,2) == 2):
                    self.set_state(WAIT_FOR_OPPOSITE)
        elif (self.state == State.WAIT_FOR_RIGHT or
            self.state == State.WAIT_FOR_OPPOSITE):
            rospy.sleep(self.T_CROSS + self.T_SENSE)
            self.set_state(State.AT_STOP_CLEARING)
        elif self.state == State.GO:
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)
        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetectionETHZ17.GO:
                self.set_state(State.GO)



    #approach 2(left bot visible)
    #SIGNAL_A = YELLOW, SIGNAL_B = GREEN, SIGNAL_C = RED
    def reconsider(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                self.reset_signals_detection()
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    self.set_state(State.NO_COLOR_QUEUE)
                    #self.set_state(State.AT_STOP_CLEARING)
        elif self.state == State.NO_COLOR_QUEUE:#you just arrived at the intersection and have no color(signal)
            if(self.right_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.right_veh == SignalsDetectionETHZ17.SIGNAL_B or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_B):
                self.set_state(State.RED_QUEUE)
            elif(self.right_veh == SignalsDetectionETHZ17.SIGNAL_C or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C or
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_C):
                self.set_state(State.NO_COLOR_QUEUE)
                #?? can if statement here go into a very long loop ??
            else:
                self.set_state(State.NEGOTIATION_QUEUE)
        elif self.state == State.RED_QUEUE:#you are in the red queue, waiting for current negotiation to end
            if(self.right_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.right_veh == SignalsDetectionETHZ17.SIGNAL_B or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_B or
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_B):
                self.set_state(State.RED_QUEUE)
            else:
                if(self.right_veh == SignalsDetectionETHZ17.SIGNAL_C or 
                    self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_C or
                    self.left_veh == SignalsDetectionETHZ17.SIGNAL_C):
                    self.state(State.NEGOTIATION_QUEUE)
                else:#if no signal from any side, enter the intersection
                    self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
                    self.set_state(State.GO)
            #todo: if two reds waiting for yellow, and yellow is gone:
            #   if one red turns green before second can catch its red color, 
            #bots from no_color_queue will join the red
            #   this might lead to starvation
            #   maybe keep track of spots of old green/yellow bots so that red 
            #seeing a new green can still turn green?
        elif self.state == State.NEGOTIATION_QUEUE:
            #self.random_delay = random() * self.T_MAX_RANDOM
            rospy.sleep(random() * self.T_MAX_RANDOM)
            if(self.right_veh == SignalsDetectionETHZ17.SIGNAL_A or 
                self.opposite_veh == SignalsDetectionETHZ17.SIGNAL_A or
                self.left_veh == SignalsDetectionETHZ17.SIGNAL_A):
                rospy.sleep(self.T_CROSS + self.T_SENSE)
                #or set state to negotiation_queue and do: 
                #if self.time_at_current_state() > self.T_CROSS + self.T_SENSE:         
                self.set_state(State.NEGOTIATION_QUEUE)
            else:#no negotiators about to go(yellow)
                self.roof_light = CoordinationSignalETHZ17.SIGNAL_A
                rospy.sleep(self.T_CROSS + self.T_SENSE)
                if(self.right_veh != SignalsDetectionETHZ17.SIGNAL_A or 
                    self.opposite_veh != SignalsDetectionETHZ17.SIGNAL_A or
                    self.left_veh != SignalsDetectionETHZ17.SIGNAL_A):
                    self.set_state(State.GO)
                else:
                    rospy.sleep(self.T_CROSS + self.T_SENSE)
                    self.set_state(State.NEGOTIATION_QUEUE)
        elif self.state == State.GO:
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)
        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetectionETHZ17.GO:
                self.set_state(State.GO)



    #approach 3
    #SIGNAL_A = CHECKING, SIGNAL_B = WAIT
    def reconsider3(self):
        if self.state == State.LANE_FOLLOWING:
            if self.mode == 'COORDINATION':
                self.reset_signals_detection()
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                else:
                    self.set_state(State.CHECKING)
        elif self.state == State.CHECKING:
            if(False):#anyone in the intersection(??? how do we check this?)
                self.set_state(State.WAIT)
            elif(self.right_veh != CoordinationSignalETHZ17.SIGNAL_A and
                self.opposite_veh != CoordinationSignalETHZ17.SIGNAL_A):
                self.set_state(State.GO)
            elif(self.right_veh == CoordinationSignalETHZ17.SIGNAL_A or
                self.opposite_veh == CoordinationSignalETHZ17.SIGNAL_A):
                self.set_state(State.WAIT)
        elif self.state == State.WAIT:
            rospy.sleep(random() * self.T_MAX_RANDOM)
            self.set_state(State.CHECKING)
        elif self.state == State.GO:
            if self.mode == 'LANE_FOLLOWING':
                self.set_state(State.LANE_FOLLOWING)
        elif self.state == State.TL_SENSING:
            if self.traffic_light == SignalsDetectionETHZ17.GO:
                self.set_state(State.GO)


#^^^^^^^^^^^^^^^^ end of reconsider methods ^^^^^^^^^^^^^^^^^^^^66

if __name__ == '__main__':
    car = VehicleCoordinator()
    rospy.spin()

