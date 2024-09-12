#!/usr/bin/env python3


from duckietown.dtros.dtros import DTROS, NodeType
import tf
import cv2
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Range
from cv_bridge import CvBridge
import tf.transformations


class RigidTransformNode(DTROS):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publisher:
    ~pose

    Subscribers:
    ~reset_transform
    ~position_control
    """
    def __init__(self, node_name):
        super(RigidTransformNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # initialize the DTROS parent class
        
        self.bridge = CvBridge()

        # initialize the Pose data
        self.pose_msg = PoseStamped()
        self.altitude = 0.3
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0

        # position hold is initialized as False
        self.position_control = True    # TODO: this flag is pretty useless, drop it
        self.first_image = None
        self.previous_image = None

        # used as a safety check for position control
        self.consecutive_lost_counter = 0
        self.lost = False

        # first image vars
        self.first = True
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None
        self.previous_points = None

        # ROS Setup
        ###########
        # Publisher
        self._posepub = rospy.Publisher('~pose', PoseStamped, queue_size=1)
        self._lostpub = rospy.Publisher('~lost', Bool, queue_size=1)
        self._debug_img_pub = rospy.Publisher("~debug_image", CompressedImage, queue_size=10)
        # Subscribers
        self._isub = rospy.Subscriber("~image/compressed", CompressedImage, self.image_callback, queue_size=1)

        # TODO: these should be services
        self._rtsub = rospy.Subscriber("~reset_transform", Empty, self.reset_callback, queue_size=1)
        self._pcsub = rospy.Subscriber("~position_control", Bool, self.position_control_callback, queue_size=1)
        
        # TODO: the _sub_alt should not take the range from the ToF sensor but from the state
        # In here we use two different sources of altitude information (which is not ideal), we should
        # either drop one or at least implement a safeguard to pick one over the other
        self._sub_alt = rospy.Subscriber('~range', Range, self.altitude_cb, queue_size=1)
        self._stsub = rospy.Subscriber("~state", Odometry, self.state_callback, queue_size=1)

        self.altitude = 0.03 # initialize to a bit off the ground
        self.altitude_ts = rospy.Time.now()
        

    def altitude_cb(self, msg : Range):
        """
        The altitude of the robot
        Args:
            msg:  the message publishing the altitude

        """
        self.altitude = msg.range
        self.altitude_ts = msg.header.stamp


    def image_callback(self, msg: CompressedImage):
        """
        A method that is called every time an image is received.
        This function implements visual odometry by estimating the motion between consecutive frames and integrating it.
        """
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.check_altitude_timeout()

        if self.position_control:
            if self.previous_image is None:
                # Store the first image to initialize odometry
                self.previous_image = image
                self.previous_points = self.detect_features(image)
                self.last_frame_time = rospy.get_time()
            else:
                # Process the current image
                self.process_image(image)

        self.publish_lost_status()

    def check_altitude_timeout(self):
        """Check if altitude data has been received recently."""
        duration_from_last_altitude = rospy.Time.now() - self.altitude_ts
        if duration_from_last_altitude.to_sec() > 10:
            self.logwarn(f"No altitude received for {duration_from_last_altitude.to_sec():10.4f} seconds.")

    def process_image(self, image):
        """Process the current image and estimate motion between consecutive frames."""
        current_points = self.detect_features(image)
        
        # # Specify the number of iterations.
        # number_of_iterations = 5000
        # # Specify the threshold of the increment
        # # in the correlation coefficient between two iterations
        # termination_eps = 1e-10
        # criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)
        # # Estimate transformation from the previous image to the current image
        # ok, transform, = cv2.findTransformECC(self.previous_image, image, None, cv2.MOTION_TRANSLATION, criteria)

        transform, inliers = cv2.estimateAffinePartial2D(self.previous_points, current_points, False)
        self.logdebug(f"Pixel translation: {transform[0, 2]}, {transform[1, 2]}")
        # If the transformation is successful, update pose
        if transform is not None:
            self.lost = False
            self.update_pose_with_transform(transform)
        else:
            self.handle_lost_image()

        # Update the previous image and points for the next frame
        self.previous_image = image
        self.previous_points = current_points

        if self._debug_img_pub.get_num_connections() > 0:
            self.publish_debug_image(image, current_points)
            
    def publish_debug_image(self, image, points):
        """
        Publish the debug image with the detected features.
        """
        # Draw the detected features
        for point in points:
            x, y = point.ravel()
            cv2.circle(image, (int(x), int(y)), 3, (0, 255, 0), -1)

        # Publish the debug image
        msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
        # msg.header.stamp = rospy.Time.now()
        self._debug_img_pub.publish(msg)

    def detect_features(self, image):
        """Detect key features in the image."""
        if self.previous_points is None:
            nextPts = cv2.goodFeaturesToTrack(image, maxCorners=10, qualityLevel=0.01, minDistance=8)
            return nextPts
        
        nextPts, status, err = cv2.calcOpticalFlowPyrLK(self.previous_image, image, self.previous_points, None)
        
        return nextPts

    def update_pose_with_transform(self, transform):
        """Update the pose using the transformation between consecutive frames."""
        # Calculate the x, y translation and yaw from the transformation
        translation, yaw = self.translation_and_yaw(transform)
        
        # Integrate the translation to update position (TODO: why do we multiply by altitude? It should actually be a normalized pixel to meter conversion)
        self.pose_msg.pose.position.x += translation[0] * self.altitude
        self.pose_msg.pose.position.y += translation[1] * self.altitude

        # Update the yaw (orientation)
        _, _, z, w = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.pose_msg.pose.orientation.z = z
        self.pose_msg.pose.orientation.w = w

        self.logdebug(f"Updated pose: x={self.pose_msg.pose.position.x}, y={self.pose_msg.pose.position.y}, yaw={yaw}")

    def handle_lost_image(self):
        """Handle the situation when the transformation fails (i.e., image is lost)."""
        self.logwarn("Lost image!")
        if self.lost:
            self.consecutive_lost_counter += 1
        else:
            self.lost = True

    def publish_lost_status(self):
        """Publish lost status if the image is lost consecutively."""
        if self.lost and self.consecutive_lost_counter >= 10:
            self._lostpub.publish(True)
            self.consecutive_lost_counter = 0
        else:
            self.consecutive_lost_counter = 0
            self._lostpub.publish(False)

        # Always publish the current pose
        self.pose_msg.header.stamp = rospy.Time.now()
        self._posepub.publish(self.pose_msg)
    
    # normalize image
    def translation_and_yaw(self, transform: np.ndarray):
        #######################################
        # TODO: we are using a hardcoded value of 640 and 480, we should use the actual image size retrieved from the `camera_info` topic
        # also the image is now rotated so the x and y are swapped
        # BASICALLY figure out what this thing is doing and adapt it to the new camera setup (or rather make it camera agnostic)
        #######################################
        
        tx = float(transform[0, 2])
        ty = float(transform[1, 2])
        
        translation_x_y = [
            ty / 640,
            tx / 480,
        ]

        # yaw jumps can be up to ~ 20 deg
        yaw_scale = np.sqrt(transform[0, 0] ** 2 + transform[1, 0] ** 2)
        yaw = np.arctan2(
            float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale
        )

        return translation_x_y, yaw

    # subscribe ~reset_transform
    # TODO: this should really be a service
    def reset_callback(self, _):
        """ Reset the current position and orientation """
        self.loginfo("Resetting Pose")

        # reset position control variables
        self.first = True

        # reset first image vars
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # reset the pose values
        self.pose_msg = PoseStamped()

        self._lostpub.publish(False)
        self.loginfo("Pose reset complete")

    # subscribe /pidrone/position_control
    # TODO: this should really be a service
    def position_control_callback(self, msg : Bool):
        ''' Set whether the pose is calculated and published '''
        self.position_control = msg.data
        self.loginfo(f"Position Control {self.position_control}")

    def state_callback(self, msg : Odometry):
        """
        Store z position (altitude) reading from state Odometry, along with most recent
        x and y position estimate
        """
        self.altitude = msg.pose.pose.position.z
        self.x_position_from_state = msg.pose.pose.position.x
        self.y_position_from_state = msg.pose.pose.position.y
        
    
def main():
    rigid_transform_node = RigidTransformNode("rigid_transform_node")
    rospy.spin()

if __name__ == '__main__':
    main()