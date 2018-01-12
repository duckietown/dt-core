#!/usr/bin/env python
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, LanePose, BoolStamped, Twist2DStamped
import duckietown_utils as dtu
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class LaneFilterNode(object):

    def __init__(self):
        self.node_name = "Lane Filter"
        self.active = True
        self.filter = None
        self.updateParams(None)

        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()
        self.d_median = []
        self.phi_median = []

        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        # Publishers
        self.pub_lane_pose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)

        self.pub_ml_img = rospy.Publisher("~ml_img", Image, queue_size=1)
        self.pub_entropy = rospy.Publisher("~entropy", Float32, queue_size=1)
        self.pub_in_lane = rospy.Publisher("~in_lane", BoolStamped, queue_size=1)

        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            self.filter = dtu.instantiate(c[0], c[1])

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processSegments(self, segment_list_msg):
        if not self.active:
            return

        # Step 1: predict
        current_time = rospy.get_time()
        dt = current_time - self.t_last_update
        v = self.velocity.v
        w = self.velocity.omega

        self.filter.predict(dt=dt, v=v, w=w)
        self.t_last_update = current_time

        # Step 2: update

        self.filter.update(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimateList()
        #print "d_max = ", d_max
        #print "phi_max = ", phi_max
#        sum_phi_l = np.sum(phi_max[1:self.filter.num_belief])
#        sum_d_l = np.sum(d_max[1:self.filter.num_belief])
#        av_phi_l = np.average(phi_max[1:self.filter.num_belief])
#        av_d_l = np.average(d_max[1:self.filter.num_belief])

        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max

        #if (sum_phi_l<-1.6 and av_d_l>0.05):
        #    print "I see a left curve"
        #elif (sum_phi_l>1.6 and av_d_l <-0.05):
        #    print "I see a right curve"
        #else:
        #    print "I am on a straight line"

        delta_dmax = np.median(d_max[1:])  # - d_max[0]
        delta_phimax = np.median(phi_max[1:])  #- phi_max[0]

        if len(self.d_median) >= 5:
            self.d_median.pop(0)
            self.phi_median.pop(0)
        self.d_median.append(delta_dmax)
        self.phi_median.append(delta_phimax)

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max[0]
        lanePose.phi = phi_max[0]
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        #print "Delta dmax", delta_dmax
        #print "Delta phimax", delta_phimax

        CURVATURE_LEFT = 0.025
        CURVATURE_RIGHT = -0.054
        CURVATURE_STRAIGHT = 0

        if np.median(self.phi_median) < -0.3 and np.median(self.d_median) > 0.05:
            print "left curve"
            lanePose.curvature = CURVATURE_LEFT
        elif np.median(self.phi_median) > 0.2 and np.median(self.d_median) < -0.02:
            print "right curve"
            lanePose.curvature = CURVATURE_RIGHT
        else:
            print "straight line"
            lanePose.curvature = CURVATURE_STRAIGHT

        # publish the belief image
        belief_img = self.getDistributionImage(self.filter.belief, segment_list_msg.header.stamp)
        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg)

    def getDistributionImage(self, mat, stamp):
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg((255 * mat).astype('uint8'), "mono8")
        img.header.stamp = stamp
        return img

    def updateVelocity(self, twist_msg):
        self.velocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter', anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
