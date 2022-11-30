#!/usr/bin/env python3

import rospy
from BaseComNode import BaseComNode
from UtilStates import ActionState
from duckietown.dtros import DTROS, NodeType, TopicType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern, BoolStamped, Twist2DStamped
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest, ChangePattern
from duckietown.dtros.utils import apply_namespace
from std_msgs.msg import String


class CommunicationNode(DTROS, BaseComNode):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        DTROS.__init__(self, node_name=node_name, node_type=NodeType.COMMUNICATION)
        BaseComNode.__init__(self, 60, 40)

        self.last_time_sec = rospy.get_time()
        self.begin_time_sec = self.last_time_sec

        # TODO Parameters
        #self._some_param = rospy.get_param("~some_parameter", False)

        # Subscribing
        self.sub = rospy.Subscriber('~image_in', CompressedImage, self.img_callback)
        # TODO subscribe to the intersection type TL or StopSign, use intersection_type_callback()

        # Publishing
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.pub_timed_out = rospy.Publisher("~timed_out", BoolStamped, queue_size=1)
        self.pub_coord_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        

        # Used Services
        #self.serv = rospy.ServiceProxy('led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
        self.changeCustomPattern = rospy.ServiceProxy(
            apply_namespace('~set_custom_pattern', ns_level=1),
            SetCustomLEDPattern
        )
        self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)
        self.currLedStateSub = rospy.Subscriber("~current_led_state", String, self.curr_led_state_callback)

        # TODO DEBUG:
        self.intersection_type_callback(1) # simulate stop sign
        # self.intersection_type_callback(2) # simulate TL

    # LED state callback for debugging, but can also be useful in case other nodes
    # change the LED while this node is running.
    def curr_led_state_callback(self, msg):
        print(f"CommunicationNode: LED color now {msg.data}")
        # TODO Handle unexpected LED changes? (by other nodes)

    def blink_at(self, frequency: int = 0):
        # TODO : These are some tests to debug the led_emitter_node.
        # Currently for some obscure reason the led_emitter_node is never accessible
        # in the container, opposed to the camera_node which is available regardless
        # how the node is launched. Both are supposed to be running when the "dt-car_interface"
        # is launched.

        # TEST set_pattern solid color
        BaseComNode.blink_at(self, frequency)
        if frequency == 0:
            msg = String()
            #self.changePattern(msg)

#        self.serv(SetCustomLEDPatternRequest(pattern=LEDPattern(
#            frequency=frequency,
#            color_list=['white'] * 5,
#            color_mask=[1] * 5,
#            frequency_mask=[1] * 5,
#        )))
        # Build LED message
#        pattern_msg = LEDPattern(
#            frequency=frequency,
#            color_list=['white'] * 5,
#            color_mask=[1] * 5,
#            frequency_mask=[1] * 5,
#        )
#        self.changeCustomPattern(pattern_msg)

        pass

    def publish_signal(self, action):
        if action is ActionState.Go:
            message = BoolStamped()
            message.header.stamp = rospy.Time.now()
            message.data = True
            self.pub_intersection_go.publish(message)
            rospy.loginfo(f"[{self.node_name}] -> Go")
        elif action == ActionState.TimedOut:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_timed_out.publish(msg)
            rospy.loginfo(f"[{self.node_name}] -> Timed-out")

    def run(self):
        rate = rospy.Rate(0.5)  # 1Hz
        while not rospy.is_shutdown():
            BaseComNode.run(self)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = CommunicationNode(node_name='communication_node')
    node.run()
    # keep spinning
    rospy.spin()
