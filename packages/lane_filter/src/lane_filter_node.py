#!/usr/bin/env python3

# USES THE NEW WHEEL ENCODERS DATA

import json
import os
import numpy as np

import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import FSMState, LanePose, SegmentList, Twist2DStamped, BoolStamped, WheelEncoderStamped, EpisodeStart
from lane_filter import LaneFilterHistogram
from sensor_msgs.msg import Image
from std_msgs.msg import String


class LaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the
    center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~(left/right)_wheel_encoder_node/tick (:obj: `WheelEncoderStamped`): Information from the wheel encoders\
        ~episode_start (:obj: `EpisodeStart`): The signal that a new episode has started - used to reset the filter

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~belief_img (:obj:`Image`): A debug image that shows the filter's internal state
    """

    filter: LaneFilterHistogram
    bridge: CvBridge

    def __init__(self, node_name):
        super(LaneFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)
        self._predict_freq = rospy.get_param("~predict_frequency", 30.0)
        #Enocder Init
        self.right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks_delta = 0
        self.left_encoder_ticks_delta = 0

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        # Load the needed filter parameters defined elsewhere need here
        try:
            self.filter.encoder_resolution = rospy.get_param("left_wheel_encoder_node/resolution")
            self.filter.wheel_baseline = rospy.get_param("kinematics_node/baseline")
            self.filter.wheel_radius = rospy.get_param("kinematics_node/radius")
        except rospy.KeyError as e:
            rospy.logerror(f"[Lane filter] Unable to load required param: {e}")

        # this is only used for the timestamp of the first publication
        self.last_update_stamp = rospy.Time.now()


        # Creating cvBridge
        self.bridge = CvBridge()


        # Subscribers

        self.sub_segment_list = rospy.Subscriber(
            "~segment_list", SegmentList, self.cbProcessSegments, queue_size=1
        )

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessLeftEncoder, queue_size=1
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessRightEncoder, queue_size=1
        )

        self.sub_episode_start = rospy.Subscriber(
            f"episode_start", EpisodeStart, self.cbEpisodeStart, queue_size=1
        )

        # Publishers
        self.pub_lane_pose = rospy.Publisher(
            "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_belief_img = rospy.Publisher(
            "~belief_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )


        # Set up a timer for prediction (if we got encoder data) since that data can come very quickly
        rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbPredict)

    def cbEpisodeStart(self, msg):
        rospy.loginfo("Lane Filter Resetting")
        self.filter.initialize()

    def cbProcessLeftEncoder(self, left_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = left_encoder_msg.resolution
            self.filter.initialized = True
        self.left_encoder_ticks_delta = left_encoder_msg.data - self.left_encoder_ticks

    def cbProcessRightEncoder(self, right_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = right_encoder_msg.resolution
            self.filter.initialized = True
        self.right_encoder_ticks_delta = right_encoder_msg.data - self.right_encoder_ticks

    def cbPredict(self, event):

        # first let's check if we moved at all, if not abort
        if self.right_encoder_ticks_delta == 0 and self.left_encoder_ticks_delta == 0:
            return

        self.filter.predict(self.left_encoder_ticks_delta, self.right_encoder_ticks_delta)
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        self.publishEstimate(self.last_update_stamp)

    def cbProcessSegments(self, segment_list_msg):
        """Callback to process the segments

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        self.last_update_stamp = segment_list_msg.header.stamp

        self.filter.update(segment_list_msg.segments)

        self.publishEstimate(segment_list_msg.header.stamp)

    def publishEstimate(self, timestamp):

        [d_max, phi_max] = self.filter.getEstimate()

        # Getting the highest belief value from the belief matrix
        max_val = self.filter.getMax()
        # Comparing it to a minimum belief threshold to make sure we are certain enough of our estimate
        in_lane = max_val > self.filter.min_max

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = timestamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        if self._debug:
            self.debugOutput()
    
    #def debugOutput(self, segment_list_msg, d_max, phi_max, timestamp_before_processing):
    def debugOutput(self):
        """Creates and publishes debug messages

        OLD METHOD Args NOT USED ANYMORE:
            segment_list_msg (:obj:`SegmentList`): message containing list of filtered segments
            d_max (:obj:`float`): best estimate for d
            phi_max (:obj:``float): best estimate for phi
            timestamp_before_processing (:obj:`float`): timestamp dating from before the processing

        """
        if self._debug:
            ### NOT USEFUL ANYMORE AS ALL FUNCTIONS ARE DECOUPLED
            # Latency of Estimation including curvature estimation
            # estimation_latency_stamp = rospy.Time.now() - timestamp_before_processing
            # estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs / 1e9
            # self.latencyArray.append(estimation_latency)

            # if len(self.latencyArray) >= 20:
            #     self.latencyArray.pop(0)

            # # print "Latency of segment list: ", segment_latency
            # self.loginfo(f"Mean latency of Estimation:................. {np.mean(self.latencyArray)}")
            ###

            ### NOT USEFUL ANYMORE AS SEGMENT LISTS ARE NO MORE PROPOGATED TO THE FUNCTION CALLING THIS
            # Get the segments that agree with the best estimate and publish them
            # inlier_segments = self.filter.get_inlier_segments(segment_list_msg.segments, d_max, phi_max)
            # inlier_segments_msg = SegmentList()
            # inlier_segments_msg.header = segment_list_msg.header
            # inlier_segments_msg.segments = inlier_segments
            # self.pub_seglist_filtered.publish(inlier_segments_msg)
            ###

            # Create belief image and publish it
            belief_img = self.bridge.cv2_to_imgmsg(
                np.array(255 * self.filter.belief).astype("uint8"), "mono8"
            )
            #belief_img.header.stamp = segment_list_msg.header.stamp # FIXME: REPLACE WITH ENCODER TIMESTAMPS MAYBE 
            self.pub_belief_img.publish(belief_img)

            #FIXME: USE THE Visualization of the lane filter
            #self.filter.get_plot_phi_d()

    def cbMode(self, msg):
        return  # TODO adjust self.active

    ### NOT USED ANYMORE
    # def updateVelocity(self, twist_msg):
    #     self.currentVelocity = twist_msg
    ###

    def loginfo(self, s):
        rospy.loginfo("[%s] %s" % (self.node_name, s))

if __name__ == "__main__":
    lane_filter_node = LaneFilterNode(node_name="lane_filter_node")
    rospy.spin()
