#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose, BoolStamped, Twist2DStamped
from duckietown_utils.instantiate_utils import instantiate

class LaneFilterNode(object):
    """
    
        Lane Filter Node
        
        Author: Liam Paull
        
        Inputs: SegmentList from line detector
        
        Outputs: LanePose - the d (lateral displacement) and phi (relative angle) 
        of the car in the lane
        
        For more info on algorithm and parameters please refer to the google doc:
         https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M

    """
    def __init__(self):
        self.node_name = "Lane Filter"
        self.active = True
        self.filter = None
        self.updateParams(None)
        
        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()
        
        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):

        # For propagation
        self.use_propagation = rospy.get_param("~use_propagation",False)
        if self.use_propagation:
            self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self.updateVelocity)

        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            self.filter = instantiate(c[0], c[1])
            

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processSegments(self,segment_list_msg):
        if not self.active:
            return

        self.filter.update(segment_list_msg.segments)

        [d_max,phi_max] = self.filter.getMAPEstimate()
        max_val = self.filter.getMax()
        # this is a big ugly hack - we should do more principled determination of if the output is good or not
        in_lane = max_val > self.filter.min_max and np.linalg.norm(measurement_likelihood) != 0

        
        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        lanePose.status = lanePose.NORMAL

        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.beliefRV).astype('uint8'), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp
        
        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg)

    def updateVelocity(self,twist_msg):
        self.velocity = twist_msg
        if self.use_propagation:
            current_time = rospy.get_time()
            self.filter.propagate(dt=current_time-self.t_last_update, v = self.velocity.v, w = self.velocity.w)
            self.t_last_update = current_time



    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")


    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
