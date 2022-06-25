#!/usr/bin/env python3


import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, StopLineReading, LEDPattern, FSMState
from duckietown_msgs.srv import ChangePattern, SetCustomLEDPattern
from sensor_msgs.msg import Range


class ObstacleDetectionNode(DTROS):
    """Subscribe to vehicle_detection and pedestrian_detection,
    and publish an 'or' of those booleans."""
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObstacleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()
        self._interval = 1. / 20  # Seconds
        self._header = None

        self.state = None
        self._sub_fsm_state = rospy.Subscriber("~mode", FSMState, self.cb_fsm_mode)

        self.topics_state = {}
        # to be monitored. each callback function parse the messages and set the
        lst_topics = [
            ('~vehicle_detection/detection', BoolStamped, self.cb_bool_stamped_template),
            ('~pedestrian_detection/detection', BoolStamped, self.cb_bool_stamped_template),
            ('~front_center_tof/range', Range, self.cb_tof_range),
        ]
        self._tof_threshold = 0.15  # TODO: make parameter

        self._subscribers = []
        for topic, topic_type, callback in lst_topics:
            self.topics_state[topic] = False
            self._subscribers.append(
                rospy.Subscriber(topic, topic_type, callback, topic,  queue_size=1)
            )

        self.pub_stopped_flag = rospy.Publisher("~stopped", BoolStamped, queue_size=1)
        self.pub_virtual_stop_line = rospy.Publisher("~virtual_stop_line", StopLineReading, queue_size=1)

        # Publish in a fixed frequency with a ros timer
        # Initialize Timer
        self._timer = rospy.Timer(
            rospy.Duration(self._interval),
            self.maybe_publish_stop_line
        )
        self.last_led_state = None
        self.changePattern = rospy.ServiceProxy("~set_custom_pattern", SetCustomLEDPattern)

    def cb_tof_range(self, msg: Range, topic: str):
        self.topics_state[topic] = (0 <= msg.range < self._tof_threshold)
        self._header = msg.header

    def cb_fsm_mode(self, msg: FSMState):
        self.state = msg.state

    def cb_bool_stamped_template(self, msg: BoolStamped, topic: str):
        """generic callback for detection flag"""
        self.topics_state[topic] = msg.data
        self._header = msg.header

    def maybe_publish_stop_line(self, _):
        header = self._header
        if header is None:
            return
        # self.logdebug(f'self.topics_state: {self.topics_state}')

        if any(self.topics_state.values()):
            # self.logdebug(f'Inside')
            self.publish_stop_line_msg(
                header=header,
                detected=True,
                at=True,
                # x=distance to pedestrian/vehicle...
                x=0.  # HACK: Hardcoded
            )
            self.set_leds(detection=False, stopped=True)
        else:
            # publish empty messages
            self.publish_stop_line_msg(header=header)
            self.set_leds(detection=False, stopped=False)

    def set_leds(self, detection, stopped):
        """
        Publish a service message to trigger the hazard light at the back of the robot
        """

        # Try turning on the hazard light
        try:
            msg = self.led_patterns(stopped=stopped, detection=detection)
            if msg != self.last_led_state:
                # self.logdebug(f"Setting LED pattern to {msg}")
                self.changePattern(msg)
            self.last_led_state = msg
        except Exception as e:
            self.logwarn(f"WARN: turning on LED hazard light failed. {e}")

    def publish_stop_line_msg(self, header, detected=False, at=False, x=0.0, y=0.0):
        """
        Makes and publishes a stop line message.

        Args:
            header: header of a ROS message, usually copied from an incoming message
            detected (`bool`): whether a vehicle has been detected
            at (`bool`): whether we are closer than the desired distance
            x (`float`): distance to the vehicle
            y (`float`): lateral offset from the vehicle (not used, always 0)
        """

        stop_line_msg = StopLineReading()
        stop_line_msg.header = header
        stop_line_msg.stop_line_detected = bool(detected)
        stop_line_msg.at_stop_line = bool(at)
        stop_line_msg.stop_line_point.x = int(x)
        stop_line_msg.stop_line_point.y = int(y)
        self.pub_virtual_stop_line.publish(stop_line_msg)

        """
        Remove once have the road anomaly watcher node
        """
        stopped_flag = BoolStamped()
        stopped_flag.header = header
        stopped_flag.data = bool(at)
        self.pub_stopped_flag.publish(stopped_flag)

    def led_patterns(self, stopped: bool, detection: bool) -> LEDPattern:
        msg = LEDPattern()

        # DB21+/4-LED bot LED mapping
        # 0 - front left
        # 2 - front right
        # 3 - rear right
        # 4 - rear left

        color_list = ["white", "white", "white", "red", "red"]

        msg.color_list = color_list
        msg.color_mask = []

        if stopped:
            msg.frequency = 5.0
            msg.frequency_mask = [0, 0, 0, 1, 1]
        elif detection:
            msg.frequency = 1.0
            msg.frequency_mask = [0, 0, 0, 1, 1]
        else:
            # no blinking for normal lane_following
            # ref: if self.state == "LANE_FOLLOWING":
            msg.frequency = 0.0
            msg.frequency_mask = [0]
            if self.state == "NORMAL_JOYSTICK_CONTROL":
                msg.color_list = ["white", "white", "white", "white", "white"]

        return msg


if __name__ == "__main__":
    obstacle_detection_node = ObstacleDetectionNode(node_name="obstacle_detection_node")
    rospy.spin()
