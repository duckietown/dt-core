#!/usr/bin/env python
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, LanePose, BoolStamped, Twist2DStamped, FSMState
from duckietown_utils.instantiate_utils import instantiate
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
import json
from duckietown import DTROS


class LaneFilterNode(DTROS):
    """ Generates an estimate of the lane pose.

    Using the segments extracted by the line_detector, using the `(d,phi)` vector


    """

    def __init__(self):
        node_name = "lane_filter_node"
        super(LaneFilterNode, self).__init__(node_name=node_name)

        self.parameters['~filter'] = None
        self.parameters['~debug'] = False

        self.updateParameters()

        # Create the filter
        c = self.parameters['~filter']
        assert isinstance(c, list) and len(c) == 2, c
        self.loginfo('new filter config: %s' % str(c))
        self.filter = instantiate(c[0], c[1])

        # Creating cvBridge
        self.bridge = CvBridge()

        self.t_last_update = rospy.get_time()
        self.currentVelocity = None

        self.d_median = []
        self.phi_median = []
        self.latencyArray = []

        # Subscribers
        self.sub = self.subscriber("~segment_list",
                                   SegmentList,
                                   self.cbProcessSegments,
                                   queue_size=1)

        self.sub_velocity = self.subscriber("~car_cmd",
                                            Twist2DStamped,
                                            self.updateVelocity)

        self.sub_change_params = self.subscriber("~change_params",
                                                 String,
                                                 self.cbTemporaryChangeParams)

        # Publishers
        self.pub_lane_pose = self.publisher("~lane_pose",
                                            LanePose,
                                            queue_size=1)

        self.pub_belief_img = self.publisher("~belief_img",
                                             Image,
                                             queue_size=1)

        self.pub_seglist_filtered = self.publisher("~seglist_filtered",
                                                   SegmentList,
                                                   queue_size=1)

        # FSM
        self.sub_switch = self.subscriber(
            "~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = self.subscriber(
            "~fsm_mode", FSMState, self.cbMode, queue_size=1)

    def cbTemporaryChangeParams(self, msg):
        # This weird callback changes parameters only temporarily - used in the unicorn intersection. comment from 03/2020
        data = json.loads(msg.data)
        params = data["params"]
        reset_time = data["time"]
        # Set all paramters which need to be updated
        for param_name in params.keys():
            param_val = params[param_name]
            params[param_name] = eval("self.filter." + str(param_name))
            exec("self.filter." + str(param_name) + "=" + str(param_val))

        # Sleep for reset time
        rospy.sleep(reset_time)

        # Reset parameters to old values
        for param_name in params.keys():
            param_val = params[param_name]
            exec("self.filter." + str(param_name) + "=" + str(param_val))

    def cbSwitch(self, switch_msg):
        # All calls to this message should be replaced directly by the srvSwitch
        request = SetBool()
        request.data = switch_msg.data
        self.srvSwitch(request)

    def cbProcessSegments(self, segment_list_msg):
        # Get actual timestamp for latency measurement
        timestamp_now = rospy.Time.now()

        # Step 1: predict
        current_time = rospy.get_time()
        if self.currentVelocity:
            dt = current_time - self.t_last_update
            self.filter.predict(dt=dt, v=self.currentVelocity.v,
                                w=self.currentVelocity.omega)

        self.t_last_update = current_time

        # Step 2: update

        self.filter.update(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()
        # print "d_max = ", d_max
        # print "phi_max = ", phi_max

        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max
        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        self.debugOutput(segment_list_msg, d_max, phi_max, timestamp_now)

    def debugOutput(self, segment_list_msg, d_max, phi_max, timestamp_now):
        if self.parameters['~debug']:
            # Get the segments that agree with the best estimate and publish them
            inlier_segments = self.filter.get_inlier_segments(segment_list_msg.segments,
                                                              d_max,
                                                              phi_max)
            inlier_segments_msg = SegmentList()
            inlier_segments_msg.header = segment_list_msg.header
            inlier_segments_msg.segments = inlier_segments
            self.pub_seglist_filtered.publish(inlier_segments_msg)

            # Create belief image and publish it
            belief_img = self.bridge.cv2_to_imgmsg(
                np.array(255 * self.filter.belief).astype("uint8"), "mono8")
            belief_img.header.stamp = segment_list_msg.header.stamp
            self.pub_belief_img.publish(belief_img)

            # Latency of Estimation including curvature estimation
            estimation_latency_stamp = rospy.Time.now() - timestamp_now
            estimation_latency = estimation_latency_stamp.secs + \
                estimation_latency_stamp.nsecs/1e9
            self.latencyArray.append(estimation_latency)

            if (len(self.latencyArray) >= 20):
                self.latencyArray.pop(0)

            # print "Latency of segment list: ", segment_latency
            print("Mean latency of Estimation:................. %s" %
                  np.mean(self.latencyArray))

    def cbMode(self, msg):
        return  # TODO adjust self.active

    def updateVelocity(self, twist_msg):
        self.currentVelocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
