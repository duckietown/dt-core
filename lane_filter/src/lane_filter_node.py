#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose, BoolStamped, Twist2DStamped
from duckietown_utils.instantiate_utils import instantiate

class LaneFilterNode(object):
    def __init__(self):
        self.node_name = "Lane Filter"
        self.active = True
        self.filter = None
        self.updateParams(None)
        
        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()

        # Define Constants
        self.curvature_res = self.filter.curvature_res

        # Set parameters to server
        rospy.set_param('~curvature_res', self.curvature_res) #Write to parameter server for transparancy
        
        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
      
        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)
        self.latencyArray = []


    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            self.filter = instantiate(c[0], c[1])
            

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processSegments(self,segment_list_msg):
        # Get actual timestamp for latency measurement
        timestamp_now = rospy.Time.now()

        if not self.active:
            return

        # Step 0: get values from server
        if (rospy.get_param('~curvature_res') is not self.curvature_res):
            self.curvature_res = rospy.get_param('~curvature_res')
            self.filter.updateRangeArray(self.curvature_res)

        # Step 1: predict
        current_time = rospy.get_time()
        self.filter.predict(dt=current_time-self.t_last_update, v = self.velocity.v, w = self.velocity.omega)
        self.t_last_update = current_time

        # Step 2: update
        self.filter.update(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()
        print "d_max = ", d_max
        print "phi_max = ", phi_max

        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max[0]
        lanePose.phi = phi_max[0]
        lanePose.in_lane = in_lane
        lanePose.status = lanePose.NORMAL

        if self.curvature_res > 0:
            lanePose.curvature = self.filter.getCurvature(d_max[1:], phi_max[1:])

        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.filter.beliefArray[0]).astype('uint8'), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp
        
        self.pub_lane_pose.publish(lanePose)

        # Latency of Estimation including curvature estimation
        estimation_latency_stamp = rospy.Time.now() - timestamp_now
        estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs/1e9
        self.latencyArray.append(estimation_latency)

        if (len(self.latencyArray) >= 20):
            self.latencyArray.pop(0)

        # print "Latency of segment list: ", segment_latency
        print("Mean latency of Estimation:................. %s" % np.mean(self.latencyArray))

        self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg)

    def updateVelocity(self,twist_msg):
        self.velocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")


    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()