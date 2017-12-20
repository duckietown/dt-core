#!/usr/bin/env python
import rospy
import time
from led_detection.LEDDetector import LEDDetector
from std_msgs.msg import Byte
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo, BoolStamped, SignalsDetection
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class LEDDetectorNode(object):
    def __init__(self):
        self.active = True # [INTERACTIVE MODE] Won't be overwritten if FSM isn't running, node always active
        self.first_timestamp = 0
        self.capture_finished = True
        self.tinit = None
        self.trigger = True
        self.node_state = 0
        self.data = []

        # Needed to publish images
        self.bridge = CvBridge()

        # Parameters
        self.capture_time = 0.97 # capture time

        # Node name
        self.node_name = rospy.get_name()
	
	# Traffic light
	self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT

        # Publish
        #self.pub_detections = rospy.Publisher("~raw_led_detection",LEDDetectionArray,queue_size=1)
        self.pub_image_right = rospy.Publisher("~image_detection_right",Image,queue_size=1)
        self.pub_image_front = rospy.Publisher("~image_detection_front", Image, queue_size=1)
        self.pub_detections  = rospy.Publisher("~led_detection", SignalsDetection, queue_size=1)
        self.pub_debug       = rospy.Publisher("~debug_info",LEDDetectionDebugInfo,queue_size=1)
        self.veh_name        = rospy.get_namespace().strip("/")

        # Subscribed
        self.sub_cam    = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.sub_trig   = rospy.Subscriber("~trigger",Byte, self.trigger_callback)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)

        # Parameters
        #self.protocol            = rospy.get_param("~LED_protocol")
        self.crop_rect_normalized = rospy.get_param("~crop_rect_normalized")
        self.capture_time         = rospy.get_param("~capture_time")
        self.cell_size            = rospy.get_param("~cell_size")
        self.continuous           = rospy.get_param('~continuous', True) # Detect continuously as long as active
                                                               # [INTERACTIVE MODE] set to False for manual trigger
        #self.frequencies = self.protocol['frequencies'].values()

        rospy.loginfo('[%s] Config: \n\t crop_rect_normalized: %s, \n\t capture_time: %s, \n\t cell_size: %s'%(self.node_name, self.crop_rect_normalized, self.capture_time, self.cell_size))

        # Check vehicle name
        if not self.veh_name:
            # fall back on private param passed thru rosrun
            # syntax is: rosrun <pkg> <node> _veh:=<bot-id>
            if rospy.has_param('~veh'):
                self.veh_name = rospy.get_param('~veh')

        if not self.veh_name:
            raise ValueError('Vehicle name is not set.')

        # Loginfo
        rospy.loginfo('[%s] Vehicle: %s'%(self.node_name, self.veh_name))
        rospy.loginfo('[%s] Waiting for camera image...' %self.node_name)

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        self.active = switch_msg.data
        if self.active:
            self.trigger = True

    def camera_callback(self, msg):
        if not self.active:
            return

        float_time = msg.header.stamp.to_sec()
        debug_msg  = LEDDetectionDebugInfo()

        if self.trigger:
            rospy.loginfo('[%s] GOT TRIGGER! Starting...')
            self.trigger          = False
            self.data             = []
            self.capture_finished = False
            # Start capturing images
            rospy.loginfo('[%s] Start capturing frames'%self.node_name)
            self.first_timestamp = msg.header.stamp.to_sec()
            self.tinit           = time.time()

        elif self.capture_finished:
            self.node_state = 0
            rospy.loginfo('[%s] Waiting for trigger...' %self.node_name)

        if self.first_timestamp > 0:
            # TODO sanity check rel_time positive, restart otherwise
            rel_time = float_time - self.first_timestamp

            # Capturing
            if rel_time < self.capture_time:
                self.node_state = 1
                # Capture image
                rgb = numpy_from_ros_compressed(msg)
                rgb = cv2.cvtColor(rgb,cv2.COLOR_BGRA2GRAY)
                rgb = cv2.resize(rgb, (640 * 1, 480 * 1))
		rgb = 255 - rgb
                rospy.loginfo('[%s] Capturing frame %s' %(self.node_name, rel_time))
                # Save image to data
                if np.size(self.data) == 0:
                    self.data = rgb
                else:
                    self.data = np.dstack((self.data,rgb))
                #self.data.append({'timestamp': float_time, 'rgb': rgb[:,:]})
                debug_msg.capture_progress = 100.0*rel_time/self.capture_time

            # Start processing
            elif not self.capture_finished and self.first_timestamp > 0:
                rospy.loginfo('[%s] Relative Time %s, processing' %(self.node_name, rel_time))
                self.node_state = 2
                self.capture_finished = True
                self.first_timestamp = 0
                self.sub_cam.unregister() # IMPORTANT! Explicitly ignore messages
                                          # while processing, accumulates delay otherwise!
                self.send_state(debug_msg)
                # Process image and publish results
                self.process_and_publish()

        self.send_state(debug_msg) # TODO move heartbeat to dedicated thread

    def trigger_callback(self, msg):
        self.trigger = True

    def process_and_publish(self):
        # Get sizes
        H,W,_ = self.data.shape

        # Crop image right
        imRight = self.data[H/10:2*H/3,3*W/5:W,:]

        # Crop image front
        imFront = self.data[H/10:H/2,W/8:W/2,:]

        # Setup SimpleBlobDetector parameters
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 5
        params.maxThreshold = 200
        params.thresholdStep = 10

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 400

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.8

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.05

        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)

        # Tolerance
        DTOL = 25

        # Allocate space
        FrameRight = []
        BlobsRight = []
        FrameFront = []
        BlobsFront = []

        # Number of images
        NIm = imRight.shape[2]

        # Iterate Right
        for t in range(imRight.shape[2]):
            # Detect blobs.
            keypoints = detector.detect(imRight[:, :, t])
            FrameRight.append(np.zeros((2, len(keypoints))))
            # im_with_keypoints = cv2.drawKeypoints(imRight[:, :, t], keypoints, np.array([]), (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # im_with_keypoints = cv2.resize(im_with_keypoints, (640*4/6*2, 480))
            # cv2.imshow("Keypoints", im_with_keypoints)
            # print(len(keypoints))

            for n in range(len(keypoints)):
                FrameRight[t][:, n] = keypoints[n].pt
                if len(BlobsRight) == 0:
                    # If no blobs saved, then save the first LED detected
                    BlobsRight.append({'p': FrameRight[t][:, n], 'N': 1, 'Signal': np.zeros(imRight.shape[2])})
                    BlobsRight[-1]['Signal'][t] = 1
                else:
                    # Thereafter, check whether the detected LED belongs to a blob
                    Distance = np.empty(len(BlobsRight))
                    for k in range(len(BlobsRight)):
                        Distance[k] = np.linalg.norm(BlobsRight[k]['p'] - FrameRight[t][:, n])
                    if np.min(Distance) < DTOL:
                        # print np.min(Distance)
                        # print np.argmin(Distance)
                        if BlobsRight[np.argmin(Distance)]['Signal'][t] == 0:
                            BlobsRight[np.argmin(Distance)]['N'] += 1
                            BlobsRight[np.argmin(Distance)]['Signal'][t] = 1
                    else:
                        BlobsRight.append({'p': FrameRight[t][:, n], 'N': 1, 'Signal': np.zeros(imRight.shape[2])})
                        BlobsRight[-1]['Signal'][t] = 1

        # Iterate Front
        for t in range(imFront.shape[2]):
            # Detect blobs.
            keypoints = detector.detect(imFront[:, :, t])
            FrameFront.append(np.zeros((2, len(keypoints))))
            # im_with_keypoints = cv2.drawKeypoints(imFront[:, :, t], keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # im_with_keypoints = cv2.resize(im_with_keypoints, (640 * 4 / 6 * 2, 480))
            # cv2.imshow("Keypoints", im_with_keypoints)
            # print(len(keypoints))

            for n in range(len(keypoints)):
                FrameFront[t][:, n] = keypoints[n].pt
                if len(BlobsFront) == 0:
                    # If no blobs saved, then save the first LED detected
                    BlobsFront.append({'p': FrameFront[t][:, n], 'N': 1, 'Signal': np.zeros(imFront.shape[2])})
                    BlobsFront[-1]['Signal'][t] = 1
                else:
                    # Thereafter, check whether the detected LED belongs to a blob
                    Distance = np.empty(len(BlobsFront))
                    for k in range(len(BlobsFront)):
                        Distance[k] = np.linalg.norm(BlobsFront[k]['p'] - FrameFront[t][:, n])
                    if np.min(Distance) < DTOL:
                        # print np.min(Distance)
                        # print np.argmin(Distance)
                        if BlobsFront[np.argmin(Distance)]['Signal'][t] == 0:
                            BlobsFront[np.argmin(Distance)]['N'] += 1
                            BlobsFront[np.argmin(Distance)]['Signal'][t] = 1
                    else:
                        BlobsFront.append({'p': FrameFront[t][:, n], 'N': 1, 'Signal': np.zeros(imFront.shape[2])})
                        BlobsFront[-1]['Signal'][t] = 1

        # Extract blobs (right)
        keypointBlobRight = []
        for k in range(len(BlobsRight)):
            keypointBlobRight.append(cv2.KeyPoint(BlobsRight[k]['p'][0], BlobsRight[k]['p'][1], DTOL))

        # Extract blobs (front)
        keypointBlobFront = []
        for k in range(len(BlobsFront)):
            keypointBlobFront.append(cv2.KeyPoint(BlobsFront[k]['p'][0], BlobsFront[k]['p'][1], DTOL))

        # Images
        imPublishRight = cv2.drawKeypoints(imRight[:,:,-1], keypointBlobRight, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        imPublishFront = cv2.drawKeypoints(imFront[:,:,-1], keypointBlobFront, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Initialize detection
        self.right = SignalsDetection.NO_CAR
        self.front = SignalsDetection.NO_CAR

        # Decide whether LED or not (right)
        for i in range(len(BlobsRight)):
	    print (1.0*BlobsRight[i]['N'])/(1.0*NIm)
            if (1.0*BlobsRight[i]['N'])/(1.0*NIm) < 0.8 and (1.0*BlobsRight[i]['N'])/(1.0*NIm) > 0.2:
                self.right = SignalsDetection.SIGNAL_A
                break

        # Decide whether LED or not (front)
        for i in range(len(BlobsFront)):
	    print (1.0* BlobsFront[i]['N'])/(1.0*NIm)
            if (1.0*BlobsFront[i]['N'])/(1.0*NIm) < 0.8 and (1.0*BlobsFront[i]['N'])/(1.0*NIm) > 0.2:
                self.front = SignalsDetection.SIGNAL_A
                break
	
	# Left bot (also UNKNOWN)
	self.left = "UNKNOWN"

        # Publish results
        self.publish(imPublishRight,imPublishFront)

        # Keep going
        if self.continuous:
            self.trigger = True
            self.sub_cam = rospy.Subscriber("camera_node/image/compressed",CompressedImage, self.camera_callback)

    def publish(self,imRight,imFront):
        #  Publish image with circles
        imRightCircle_msg = self.bridge.cv2_to_imgmsg(imRight,encoding="passthrough")
        imFrontCircle_msg = self.bridge.cv2_to_imgmsg(imFront,encoding="passthrough")

        # Publish image
        self.pub_image_right.publish(imRightCircle_msg)
        self.pub_image_front.publish(imFrontCircle_msg)

        # Loginfo (right)
        if self.right != SignalsDetection.NO_CAR:
            rospy.loginfo('LED detected (right)')
        else:
            rospy.loginfo('No LED detected (right)')

        # Loginfo (front)
        if self.front != SignalsDetection.NO_CAR:
            rospy.loginfo('LED detected (front)')
        else:
            rospy.loginfo('No LED detected (front)')

        # Publish
        rospy.loginfo("[%s] The observed LEDs are:\n Front = %s\n Right = %s\n Traffic light state = %s" % (self.node_name, self.front, self.right, self.traffic_light_state))
        self.pub_detections.publish(SignalsDetection(front=self.front, right=self.right, left=self.left,traffic_light_state=self.traffic_light_state))

    def send_state(self, msg):
        msg.state = self.node_state
        self.pub_debug.publish(msg)

if __name__ == '__main__':
    rospy.init_node('LED_detector_node',anonymous=False)
    node = LEDDetectorNode()
    rospy.spin()


