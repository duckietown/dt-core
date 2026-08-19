#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown.dtros.utils import apply_namespace
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern
from std_srvs.srv import SetBool


class TrafficLightNode(DTROS):
    """Cycles traffic-light LEDs through green and all-red phases."""

    def __init__(self, node_name: str):
        super(TrafficLightNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.COMMUNICATION,
        )

        self._number_leds = rospy.get_param("~number_leds")
        self._activation_order = rospy.get_param("~activation_order")
        self._green_time = rospy.get_param("~green_time")
        self._all_red_time = rospy.get_param("~all_red_time")
        self._frequency = rospy.get_param("~frequency")
        self._green_idx = 0
        self._color_mask = [1] * self._number_leds + [0] * (5 - self._number_leds)
        emitter_switch_service = apply_namespace(
            "led_emitter_node/switch",
            ns_level=1,
        )
        rospy.wait_for_service(emitter_switch_service)
        self._emitter_switch = rospy.ServiceProxy(
            emitter_switch_service,
            SetBool,
        )
        self._emitter_switch(data=True)
        self._change_pattern = rospy.ServiceProxy(
            apply_namespace("led_emitter_node/set_custom_pattern", ns_level=1),
            SetCustomLEDPattern,
        )
        cycle_duration = self._green_time + self._all_red_time
        self._traffic_cycle = rospy.Timer(
            rospy.Duration(cycle_duration),
            self._change_direction,
        )

        self.log("Initialized.")

    def _change_direction(self, _event):
        self._green_idx = (self._green_idx + 1) % self._number_leds
        green_led = self._activation_order[self._green_idx]
        frequency_mask = [0] * 5
        frequency_mask[green_led] = 1
        color_list = ["red"] * 5
        color_list[green_led] = "green"

        pattern = LEDPattern()
        pattern.color_list = self._to_led_order(color_list)
        pattern.color_mask = self._color_mask
        pattern.frequency = self._frequency
        pattern.frequency_mask = self._to_led_order(frequency_mask)
        self._change_pattern(pattern)

        rospy.sleep(self._green_time)

        pattern.color_list = ["red"] * 5
        pattern.frequency = 0
        self._change_pattern(pattern)

    @staticmethod
    def _to_led_order(unordered_list):
        ordering = [0, 4, 1, 3, 2]
        return [unordered_list[index] for index in ordering]


if __name__ == "__main__":
    TrafficLightNode(node_name="traffic_light")
    rospy.spin()
