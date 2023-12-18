#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import DroneControl

from duckietown.dtros import DTParam, DTROS, NodeType, ParamType


class FlyCommandsMuxNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(FlyCommandsMuxNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Initialize the parameters
        self.frequency = rospy.get_param("~publish_frequency")
        self.r_override = DTParam("~roll_override", param_type=ParamType.BOOL)
        self.p_override = DTParam("~pitch_override", param_type=ParamType.BOOL)
        self.y_override = DTParam("~yaw_override", param_type=ParamType.BOOL)
        self.t_override = DTParam("~throttle_override", param_type=ParamType.BOOL)
        self.command_ttl = rospy.get_param("~command_ttl")

        # different sources of commands
        self.manual_commands = None
        self.autonomous_commands = None
        # to check if command is old
        self.last_stamp_manual = rospy.Time.now()
        self.last_stamp_autonomous = rospy.Time.now()

        # subscribers
        rospy.Subscriber("~commands/manual", DroneControl, self.cb_manual, queue_size=1)
        rospy.Subscriber("~commands/autonomous", DroneControl, self.cb_autonomous, queue_size=1)

        # publishers
        self.pub_cmds = rospy.Publisher("~commands/output", DroneControl, queue_size=1)

        # timer
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.frequency),
            self.publish_fly_commands,
        )

        self.log("Initialization completed.")

    def cb_manual(self, msg):
        self.last_stamp_manual = rospy.Time.now()
        self.manual_commands = msg

    def cb_autonomous(self, msg):
        self.last_stamp_autonomous = rospy.Time.now()
        self.autonomous_commands = msg

    def publish_fly_commands(self, _):
        # discard old commands
        t_now = rospy.Time.now().to_sec()
        if t_now - self.last_stamp_manual.to_sec() > self.command_ttl:
            self.manual_commands = None
        if t_now - self.last_stamp_autonomous.to_sec() > self.command_ttl:
            self.autonomous_commands = None

        if self.manual_commands is None:
            if self.autonomous_commands is None:
                # no valid input, do not publish
                return
            else:  # only auto commands
                self.pub_cmds.publish(self.autonomous_commands)
        else:
            if self.autonomous_commands is None:
                self.pub_cmds.publish(self.manual_commands)
            else:  # when both are valid
                msg = DroneControl()
                self.logdebug((
                    "Masks (r p y t): "
                    f"{self.r_override.value} "
                    f"{self.p_override.value} "
                    f"{self.y_override.value} "
                    f"{self.t_override.value}"
                ))
                msg.roll = self.manual_commands.roll if self.r_override.value \
                    else self.autonomous_commands.roll
                msg.pitch = self.manual_commands.pitch if self.p_override.value \
                    else self.autonomous_commands.pitch
                msg.yaw = self.manual_commands.yaw if self.y_override.value \
                    else self.autonomous_commands.yaw
                msg.throttle = self.manual_commands.throttle if self.t_override.value \
                    else self.autonomous_commands.throttle
                self.pub_cmds.publish(msg)


if __name__ == "__main__":
    node = FlyCommandsMuxNode("fly_commands_mux_node")
    rospy.spin()
