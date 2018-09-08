#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped

class StopNode(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up publishers
        self.pub_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

        self.rate = rospy.Rate(10)

        self.MainLoop()

    def MainLoop(self):
        while not rospy.is_shutdown():
            msg = Twist2DStamped()
            msg.header.stamp = rospy.Time.now()
            msg.v = 0
            msg.omega = 0
            self.pub_cmd.publish(msg)

            self.rate.sleep()

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('stop_node', anonymous=False)

    # create the intersection navigation object
    node = StopNode()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    rospy.spin()
