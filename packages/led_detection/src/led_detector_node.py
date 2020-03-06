#!/usr/bin/env python
import rospy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import scipy.fftpack

from duckietown import DTROS, DTPublisher, DTSubscriber
from duckietown_utils.bag_logs import numpy_from_ros_compressed

from std_msgs.msg import Byte
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray,\
                                BoolStamped, SignalsDetection
from sensor_msgs.msg import CompressedImage


class LEDDetectorNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LEDDetectorNode, self).__init__(node_name=node_name)

        # Needed to publish images
        self.bridge = CvBridge()

        # Initialize detectors
        self.detector_car = None
        self.detector_tl = None

        # Add the node parameters to the parameters dictionary
        self.parameters['~capture_time'] = None
        self.parameters['~DTOL'] = None
        self.parameters['~useFFT'] = None
        self.parameters['~freqIdentity'] = None
        self.parameters['~crop_params'] = None
        self.parameters['~blob_detector_db'] = None
        self.parameters['~blob_detector_tl'] = None
        self.parameters['~verbose'] = None
        self.parameters['~cell_size'] = None
        self.parameters['~LED_protocol'] = None

        # To trigger the first change, we set this manually
        self.parameterChanged = True
        self.updateParameters()

        self.first_timestamp = 0
        self.capture_finished = True
        self.t_init = None
        self.trigger = True
        self.node_state = 0
        self.data = []

        # Initialize detection
        self.right = None
        self.front = None
        self.traffic_light = None
        # We currently are not able to see what happens on the left
        self.left = "UNKNOWN"

        # Publishers
        self.pub_detections = DTPublisher("~signals_detection", SignalsDetection, queue_size=1)

        # Publishers for debug images
        self.pub_image_right = DTPublisher("~image_detection_right/compressed", CompressedImage, queue_size=1)
        self.pub_image_front = DTPublisher("~image_detection_front/compressed", CompressedImage, queue_size=1)
        self.pub_image_TL = DTPublisher("~image_detection_TL/compressed", CompressedImage, queue_size=1)

        # Subscribers
        self.sub_cam = DTSubscriber("~image/compressed", CompressedImage, self.camera_callback)

        # Log info
        self.log('Initialized!')

    def camera_callback(self, msg):

        float_time = msg.header.stamp.to_sec()

        if self.trigger:
            self.trigger = False
            self.data = []
            self.capture_finished = False
            # Start capturing images
            self.first_timestamp = msg.header.stamp.to_sec()
            self.t_init = rospy.Time.now().to_sec()

        elif self.capture_finished:
            self.node_state = 0

        if self.first_timestamp > 0:
            rel_time = float_time - self.first_timestamp

            # Capturing
            if rel_time < self.parameters['~capture_time']:
                self.node_state = 1
                # Capture image
                rgb = numpy_from_ros_compressed(msg)
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGRA2GRAY)
                rgb = 255 - rgb
                self.data.append({'timestamp': float_time, 'rgb': rgb[:, :]})

            # Start processing
            elif not self.capture_finished and self.first_timestamp > 0:
                if self.parameters['~verbose'] == 2:
                    self.log('Relative Time %s, processing' % rel_time)
                self.node_state = 2
                self.capture_finished = True
                self.first_timestamp = 0

                # IMPORTANT! Explicitly ignore messages while processing, accumulates delay otherwise!
                self.sub_cam.unregister()

                # Process image and publish results
                self.process_and_publish()

    def process_and_publish(self):
        # Initial time
        tic = rospy.Time.now().to_sec()

        # Get dimensions
        h, w = self.data[0]['rgb'].shape
        num_img = len(self.data)

        # Save images in numpy arrays
        images = np.zeros((h, w, num_img), dtype=np.uint8)
        timestamps = np.zeros(num_img)
        for i, v in enumerate(self.data):
            timestamps[i] = v['timestamp']
            images[:, :, i] = v['rgb']

        # Crop images
        img_right = self.crop_image(images, self.parameters['~crop_params']['cropNormalizedRight'])
        img_front = self.crop_image(images, self.parameters['~crop_params']['cropNormalizedFront'])
        img_tl = self.crop_image(images, self.parameters['~crop_params']['cropNormalizedTL'])

        # Print on screen
        if self.parameters['~verbose'] == 2:
            self.log('Analyzing %s images of size %s X %s' % (num_img, w, h))

        # Get blobs right
        blobs_right, frame_right = self.get_blobs(img_right, self.detector_car)
        # Get blobs front
        blobs_front, frame_front = self.get_blobs(img_front, self.detector_car)
        # Get blobs right
        blobs_tl, frame_tl = self.get_blobs(img_tl, self.detector_tl)

        radius = self.parameters['~DTOL']/2.0

        if self.parameters['~verbose'] > 0:
            # Extract blobs for visualization
            keypoint_blob_right = self.extract_blobs(blobs_right, radius)
            keypoint_blob_front = self.extract_blobs(blobs_front, radius)
            keypoint_blob_tl = self.extract_blobs(blobs_tl, radius)

            # Images
            img_pub_right = cv2.drawKeypoints(img_right[:, :, -1], keypoint_blob_right, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            img_pub_front = cv2.drawKeypoints(img_front[:, :, -1], keypoint_blob_front, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            img_pub_tl = cv2.drawKeypoints(img_tl[:, :, -1], keypoint_blob_tl, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            img_pub_right = None
            img_pub_front = None
            img_pub_tl = None

        # Initialize detection
        self.right = None
        self.front = None
        self.traffic_light = None

        # Sampling time
        t_s = (1.0*self.parameters['~capture_time'])/(1.0*num_img)

        # Decide whether LED or not
        self.right = self.is_signal(blobs_right, t_s, num_img)
        self.front = self.is_signal(blobs_front, t_s, num_img)
        self.traffic_light = self.is_signal(blobs_tl, t_s, num_img)

        # Left bot (also UNKNOWN)
        self.left = "UNKNOWN"

        # Final time
        processing_time = rospy.Time.now().to_sec() - tic
        total_time = rospy.Time.now().to_sec() - self.t_init

        # Publish results
        self.publish(img_pub_right, img_pub_front, img_pub_tl)

        # Print performance
        if self.parameters['~verbose'] == 2:
            self.log('[%s] Detection completed. Processing time: %.2f s. Total time:  %.2f s' %
                     (self.node_name, processing_time, total_time))

        # Keep going
        self.trigger = True
        self.sub_cam = DTSubscriber("~image/compressed", CompressedImage, self.camera_callback)

    def get_blobs(self, images, detector):

        blobs = []
        frame = []
        # Iterate over time
        num_images = images.shape[-1]
        for t in range(num_images):
            keypoints = detector.detect(images[:, :, t])
            frame.append(np.zeros((2, len(keypoints))))

            for n in range(len(keypoints)):
                frame[t][:, n] = keypoints[n].pt
                if len(blobs) == 0:
                    # If no blobs saved, then save the first LED detected
                    blobs.append({'p': frame[t][:, n], 'N': 1, 'Signal': np.zeros(images.shape[2])})
                    blobs[-1]['Signal'][t] = 1
                else:
                    # Thereafter, check whether the detected LED belongs to a blob
                    dist = np.empty(len(blobs))
                    for k in range(len(blobs)):
                        dist[k] = np.linalg.norm(blobs[k]['p'] - frame[t][:, n])
                    if np.min(dist) < self.parameters['~DTOL']:
                        if blobs[np.argmin(dist)]['Signal'][t] == 0:
                            blobs[np.argmin(dist)]['N'] += 1
                            blobs[np.argmin(dist)]['Signal'][t] = 1
                    else:
                        blobs.append({'p': frame[t][:, n], 'N': 1, 'Signal': np.zeros(images.shape[2])})
                        blobs[-1]['Signal'][t] = 1

        return blobs, frame

    @staticmethod
    def extract_blobs(blobs, radius):
        # Extract blobs
        keypoint_blob = []
        for blob in blobs:
            assert np.sum(blob['Signal']) == blobs['N']
            keypoint_blob.append(cv2.KeyPoint(blobs['p'][0], blobs['p'][1], radius))
        return keypoint_blob

    def is_signal(self, blobs, t_s, num_img):
        # Decide whether LED or not
        for blob in blobs:
            # Detection
            detected, freq_identified, fft_peak_freq = self.detect_blob(blob, t_s, num_img)
            # Take decision
            detected_signal = None
            if detected:
                if self.parameters['~verbose'] == 2:
                    msg = '\n-------------------\n' + \
                          'num_img = %d \n' % num_img + \
                          't_samp = %f \n' % t_s + \
                          'fft_peak_freq = %f \n' % fft_peak_freq + \
                          'freq_identified = %f \n' % freq_identified + \
                          '-------------------'
                    self.log(msg)

                for signal_name, signal_value in self.parameters['~LED_protocol']['signals'].items():
                    if signal_value['frequency'] == freq_identified:
                        detected_signal = signal_name

                return detected_signal

    def detect_blob(self, blob, t_s, num_img):
        """Detects if a blob is blinking at specific frequencies."""
        # Percentage of appearance
        appearance_percentage = (1.0*blob['N'])/(1.0*num_img)

        # Frequency estimation based on FFT
        signal_f = scipy.fftpack.fft(blob['Signal'] - np.mean(blob['Signal']))
        y_f = 2.0/num_img*np.abs(signal_f[:num_img/2+1])
        fft_peak_freq = 1.0*np.argmax(y_f)/(num_img*t_s)

        if self.parameters['~verbose'] == 2:
            self.log('[%s] Appearance perceived. = %s, frequency = %s' %
                     (self.node_name, appearance_percentage, fft_peak_freq))
        freq_identified = 0
        # Take decision
        detected = False
        freq_to_identify = self.parameters['~LED_protocol']['frequencies'].values()
        for freq in freq_to_identify:
            if abs(fft_peak_freq - freq) < 0.35:
                # Decision
                detected = True
                freq_identified = freq
                break

        return detected, freq_identified, fft_peak_freq

    def publish(self, img_right, img_front, img_tl):
        #  Publish image with circles if verbose is > 0
        if self.parameters['~verbose'] > 0:
            img_right_circle_msg = self.bridge.cv2_to_compressed_imgmsg(img_right) # , encoding="bgr8")
            img_front_circle_msg = self.bridge.cv2_to_compressed_imgmsg(img_front) # , encoding="bgr8")
            img_tl_circle_msg = self.bridge.cv2_to_compressed_imgmsg(img_tl) # , encoding="bgr8")

            # Publish image
            self.pub_image_right.publish(img_right_circle_msg)
            self.pub_image_front.publish(img_front_circle_msg)
            self.pub_image_TL.publish(img_tl_circle_msg)

        # Log results to the terminal
        self.log("The observed LEDs are:\n Front = %s\n Right = %s\n Traffic light state = %s" %
                 (self.front, self.right, self.traffic_light))

        # Publish detections
        detections_msg = SignalsDetection(front=self.front,
                                          right=self.right,
                                          left=self.left,
                                          traffic_light_state=self.traffic_light)
        self.pub_detections.publish(detections_msg)

    def updateParameters(self):
        """Updates parameters."""
        super(LEDDetectorNode, self).updateParameters()
        if self.parameterChanged:
            self.controller.update_parameters(self.parameters)
            # We first create the detector objects, otherwise we cannot update their parameters
            bd_param_db = cv2.SimpleBlobDetector_Params()
            bd_param_tl = cv2.SimpleBlobDetector_Params()

            # Assign values to object variables
            for key, val in self.parameters['~blob_detector_db'].items():
                setattr(bd_param_db, key, val)
            for key, val in self.parameters['~blob_detector_tl'].items():
                setattr(bd_param_tl, key, val)

            # Create a detector with the parameters
            self.detector_car = cv2.SimpleBlobDetector_create(bd_param_db)
            self.detector_tl = cv2.SimpleBlobDetector_create(bd_param_tl)

            self.parameterChanged = False


if __name__ == '__main__':
    # Initialize the node
    camera_node = LEDDetectorNode(node_name='led_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()