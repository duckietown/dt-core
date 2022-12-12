#!/usr/bin/env python3

import rospy
from BaseComNode import BaseComNode
from UtilStates import ActionState, IntersectionType
from duckietown.dtros import DTROS, NodeType, TopicType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern, BoolStamped, Twist2DStamped, AprilTagsWithInfos
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest, ChangePattern
from duckietown.dtros.utils import apply_namespace
from std_msgs.msg import String
import time
from duckietown_msgs.msg import TagInfo # For tests simulating intersection type


class CommunicationNode(DTROS, BaseComNode):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        DTROS.__init__(self, node_name=node_name, node_type=NodeType.COMMUNICATION)
        BaseComNode.__init__(self, 60, 40)

        # TODO: Parameters -> self._some_param = rospy.get_param("~some_parameter", False)

        # Subscribing
        self.img_sub = rospy.Subscriber('~image_in', CompressedImage, self.img_callback)
        # TODO Subscribe to the intersection type TL/StopSign. This needs to be mapped accordingly from the FSM/appropriate node (Detectors)
        # String: Something like "TL" for TrafficLight or "SS" for StopSign
        self.intersectype_sub = rospy.Subscriber('~intersection_type_in', AprilTagsWithInfos, self.intersection_type_callback,queue_size=1)


        # Publishing
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.pub_timed_out = rospy.Publisher("~timed_out", BoolStamped, queue_size=1)
        #self.pub_coord_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        # Used Services
        self.changeCustomPattern = rospy.ServiceProxy(
            '~set_custom_pattern',
            SetCustomLEDPattern
        )

        # TODO TESTING:
        #time.sleep(5) # Wait for LED emitter to be running.
        #new_info = TagInfo()
        #new_info.tag_type = new_info.SIGN
        #new_tag_data = AprilTagsWithInfos()        
        # simulate stop sign
        #new_info.traffic_sign_type = new_info.STOP
        #new_tag_data.infos.append(new_info)
        #self.intersection_type_callback(new_tag_data)
        # simulate TL
        #new_info.traffic_sign_type = new_info.T_LIGHT_AHEAD
        #new_tag_data.infos.append(new_info)
        #self.intersection_type_callback(new_tag_data)


    def blink_at(self, frequency: int = 0, color: str='white'):

        BaseComNode.blink_at(self, frequency, color)

        # Build LED message
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
            # Set color to solid Green for 1 sec
            self.blink_at(frequency=0, color='green')
            time.sleep(2)
            # Trun the LEDs off
            self.blink_at(frequency=0, color='switchedoff')
            time.sleep(2)

            # Reset the intersection as a last step to avoid early rest by april tag reading
            self.curr_intersection_type = IntersectionType.Unknown

            # Publish go message
            message.data = True
            self.pub_intersection_go.publish(message)
            rospy.loginfo(f"[{self.node_name}] -> Go")


            # TODO TESTING:
            #new_info = TagInfo()
            #new_info.tag_type = new_info.SIGN
            #new_tag_data = AprilTagsWithInfos()
            #new_tag_data.infos.append(new_info)
            #self.intersection_type_callback(new_tag_data) # Unknown
            # simulate stop sign
            #new_info.traffic_sign_type = new_info.STOP
            #new_tag_data.infos.append(new_info)
            #self.intersection_type_callback(new_tag_data)
            # simulate TL
            #new_info.traffic_sign_type = new_info.T_LIGHT_AHEAD
            #new_tag_data.infos.append(new_info)
            #self.intersection_type_callback(new_tag_data)

        elif action == ActionState.TimedOut:
            # Set color to red
            self.blink_at(frequency=0, color='red')
            time.sleep(2)

            # Publish timed-out message
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
