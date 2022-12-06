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

        # TODO: Parameters -> self._some_param = rospy.get_param("~some_parameter", False)

        # Subscribing
        self.sub = rospy.Subscriber('~image_in', CompressedImage, self.img_callback)
        # TODO subscribe to the intersection type TL or StopSign, use intersection_type_callback()

        # Publishing
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.pub_timed_out = rospy.Publisher("~timed_out", BoolStamped, queue_size=1)
        self.pub_coord_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        # Used Services
        self.changeCustomPattern = rospy.ServiceProxy(
            apply_namespace('~set_custom_pattern', ns_level=1),
            SetCustomLEDPattern
        )

    def blink_at(self, frequency: int = 0):
        # TODO : Replace with new system.

        BaseComNode.blink_at(self, frequency)

        # Build LED message
        color = 'white' if frequency != 0 else 'off'
        pattern_msg = LEDPattern(
            frequency=frequency,
            color_list=[color] * 5,
            color_mask=[1] * 5,
            frequency_mask=[1] * 5,
        )
        self.changeCustomPattern(pattern_msg)

    def publish_signal(self, action):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()

        if action is ActionState.Go:
            message.data = True
            self.pub_intersection_go.publish(message)
            rospy.loginfo(f"[{self.node_name}] -> Go")
        elif action == ActionState.TimedOut:
            message.data = True
            self.pub_timed_out.publish(message)
            rospy.loginfo(f"[{self.node_name}] -> Timed-out")

    def run(self):
        rate = rospy.Rate(0.5)  # 1Hz
        while not rospy.is_shutdown():
            BaseComNode.run(self)
            rate.sleep()

            # TODO Needed to hold the bot still. Does not work and make the node hang, keep commented for now
            #car_cmd_msg = Twist2DStamped(v=0.0, omega=0.0)
            #car_cmd_msg.header.stamp = current_time_stamp
            #self.pub_coord_cmd.publish(car_cmd_msg)


if __name__ == '__main__':
    # create the node
    node = CommunicationNode(node_name='communication_node')
    node.run()
    # keep spinning
    rospy.spin()
