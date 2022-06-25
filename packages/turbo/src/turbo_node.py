#!/usr/bin/env python3

import time
from typing import Optional

import rospy

from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped

from duckietown.dtros import DTROS, NodeType, DTParam


class EncoderMonitor:

    def __init__(self, name: str, sensitivity: float):
        self._sensitivity = sensitivity
        self._last_tick = 0
        self._current_tick = 0
        self._last_moved = None
        self._last_still = None
        self._subscriber = rospy.Subscriber(
            f"~{name}_wheel_encoder/tick", WheelEncoderStamped, self._on_tick, queue_size=1
        )
        self._timer = rospy.Timer(rospy.Duration(0.5), self._check)

    @property
    def last_moved(self) -> Optional[float]:
        return self._last_moved

    @property
    def last_still(self) -> Optional[float]:
        return self._last_still

    @property
    def since_last_moved(self) -> float:
        if self._last_moved is None:
            return 0
        return time.time() - self.last_moved

    @property
    def since_last_still(self) -> float:
        if self._last_still is None:
            return 0
        return time.time() - self.last_still

    def clear(self):
        self._last_moved = time.time()

    def _on_tick(self, msg):
        self._current_tick = msg.data

    def _check(self, _):
        diff = self._current_tick - self._last_tick
        if abs(diff) > self._sensitivity:
            self._last_moved = time.time()
            self._last_tick = self._current_tick
        else:
            self._last_still = time.time()


class MotorMonitor:

    def __init__(self):
        self._last_still = time.time()

    @property
    def last_still(self) -> Optional[float]:
        return self._last_still

    @property
    def since_last_still(self) -> float:
        return time.time() - self._last_still

    def clear(self):
        self._last_still = time.time()

    def set_motion(self, speed: float):
        if speed == 0.0:
            self._last_still = time.time()


class TurboNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TurboNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # define parameters
        self._maximum_gain = rospy.get_param("~maximum_gain")
        self._step = rospy.get_param("~step")
        self._period = rospy.get_param("~period")
        self._delay = rospy.get_param("~delay")
        self._encoder_sensitivity = rospy.get_param("~encoder_sensitivity")
        self._gain_param = DTParam("~gain")

        # internal state
        self._current_gain = self._gain_param.value
        self._default_gain = self._gain_param.value
        self._left_encoder = EncoderMonitor("left", self._encoder_sensitivity)
        self._right_encoder = EncoderMonitor("right", self._encoder_sensitivity)
        self._left_motor = MotorMonitor()
        self._right_motor = MotorMonitor()

        # subscribers
        self._wheels_cmd = rospy.Subscriber(
            "~wheels_cmd_executed",
            WheelsCmdStamped,
            self._on_wheels_cmd_executed_cb,
            queue_size=1
        )

        # timer
        self._timer = rospy.Timer(rospy.Duration(self._period), self._check)

    @property
    def _is_active(self):
        return self._default_gain != self._current_gain

    def _on_gain_change(self):
        new_gain = self._gain_param.value
        if new_gain == self._current_gain:
            return
        self._default_gain = new_gain
        self._current_gain = new_gain

    def _on_wheels_cmd_executed_cb(self, msg):
        self._left_motor.set_motion(msg.vel_left)
        self._right_motor.set_motion(msg.vel_right)

    def _deactivate(self):
        self._current_gain = self._default_gain
        rospy.set_param("~gain", self._default_gain)

    def _bump_gain(self, dryrun: bool = False) -> bool:
        gain = min(self._current_gain + self._step, self._maximum_gain)
        if gain == self._current_gain:
            return False
        if not dryrun:
            self._current_gain = gain
            # TODO: re-enable
            rospy.set_param("~gain", gain)
        return True

    def _check(self, _):
        # we are looking for cases in which, for the last `delay` seconds:
        #   - the motor command hasn't dropped to zero;
        #   - the encoder hasn't registered any motion;
        left_motor_moved = self._left_motor.since_last_still > self._delay
        right_motor_moved = self._right_motor.since_last_still > self._delay
        left_encoder_still = self._left_encoder.since_last_moved > self._delay
        right_encoder_still = self._right_encoder.since_last_moved > self._delay
        left_encoder_moved = self._left_encoder.since_last_still > self._delay
        right_encoder_moved = self._right_encoder.since_last_still > self._delay
        trigger: Optional[bool] = None
        # left motor is stuck
        if left_motor_moved and left_encoder_still and self._bump_gain(dryrun=True):
            self.logwarn("Detected no motion on LEFT wheel.")
            trigger = True
        # right motor is stuck
        if right_motor_moved and right_encoder_still and self._bump_gain(dryrun=True):
            self.logwarn("Detected no motion on RIGHT wheel.")
            trigger = True
        # wheel status
        left_wheel_ok = left_motor_moved and left_encoder_moved
        right_wheel_ok = right_motor_moved and right_encoder_moved

        print(left_motor_moved, left_encoder_moved, left_wheel_ok)
        print(right_motor_moved, right_encoder_moved, right_wheel_ok)
        print()

        if (left_wheel_ok and right_wheel_ok) and self._is_active:
            trigger = False
        # trigger turbo
        previous_gain = self._current_gain
        if trigger is True:
            self._bump_gain()
            self.loginfo(f"Bumping gain {previous_gain:.2f} -> {self._current_gain:.2f}")
        elif trigger is False:
            self._deactivate()
            self.loginfo(f"Motion detected, disabling turbo. "
                         f"Reducing gain {previous_gain:.2f} -> {self._current_gain:.2f}")
        else:
            # turbo remains unchanged
            pass


if __name__ == "__main__":
    # initialize the node
    turbo_node = TurboNode(node_name="turbo_node")
    # keep it spinning to keep the node alive
    rospy.spin()
