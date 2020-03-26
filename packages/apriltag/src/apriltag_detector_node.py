#!/usr/bin/env python

import cv2
import rospy
import tf
import numpy as np

from threading import Thread
from cv_bridge import CvBridge

from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType, TopicType, DTReminder, DTParam, ParamType

from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion


class AprilTagDetector(DTROS):

    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name='apriltag_detector_node',
            node_type=NodeType.PERCEPTION
        )
        # get static parameters
        self.family = rospy.get_param('~family', 'tag36h11')
        self.nthreads = rospy.get_param('~nthreads', 1)
        self.quad_decimate = rospy.get_param('~quad_decimate', 1.0)
        self.quad_sigma = rospy.get_param('~quad_sigma', 0.0)
        self.refine_edges = rospy.get_param('~refine_edges', 1)
        self.decode_sharpening = rospy.get_param('~decode_sharpening', 0.25)
        self.tag_size = rospy.get_param('~tag_size', 0.065)
        # dynamic parameter
        self.detection_freq = DTParam(
            '~detection_freq',
            default=-1,
            param_type=ParamType.INT,
            min_value=-1,
            max_value=30
        )
        self._detection_reminder = DTReminder(frequency=self.detection_freq.value)
        # camera info
        self._camera_parameters = None
        self._camera_frame = None
        # create detector object
        self._at_detector = Detector(
            families=self.family,
            nthreads=self.nthreads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=self.refine_edges,
            decode_sharpening=self.decode_sharpening
        )
        # create a CV bridge object
        self._bridge = CvBridge()
        # create subscribers
        self._img_sub = rospy.Subscriber('image_rect', Image, self._img_cb)
        self._cinfo_sub = rospy.Subscriber('camera_info', CameraInfo, self._cinfo_cb)
        # create publishers
        self._img_pub = rospy.Publisher(
            'tag_detections_image/compressed', CompressedImage, queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION
        )
        self._tag_pub = rospy.Publisher(
            'tag_detections', AprilTagDetectionArray, queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )
        self._img_pub_busy = False
        # create TF broadcaster
        self._tf_bcaster = tf.TransformBroadcaster()
        # spin forever
        rospy.spin()

    def _img_cb(self, data):
        if self._camera_parameters is None:
            return
        if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
            return
        # ---
        # turn image message into grayscale image
        img = self._bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        # detect tags
        tags = self._at_detector.detect(img, True, self._camera_parameters, self.tag_size)
        # draw the detections on an image (if needed)
        if self._img_pub.anybody_listening() and not self._img_pub_busy:
            self._img_pub_busy = True
            Thread(target=self._publish_detections_image, args=(img, tags)).start()
        # pack detections into a message
        msg = AprilTagDetectionArray()
        detection_time = rospy.Time.now()
        msg.header.stamp = detection_time
        msg.header.frame_id = self._camera_frame
        for tag in tags:
            # turn rotation matrix into quaternion
            q = _matrix_to_quaternion(tag.pose_R)
            p = tag.pose_t.T[0]
            # create single tag detection object
            detection = AprilTagDetection(
                transform=Transform(
                    translation=Vector3(
                        x=p[0],
                        y=p[1],
                        z=p[2]
                    ),
                    rotation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3]
                    )
                ),
                tag_id=tag.tag_id,
                tag_family=tag.tag_family,
                hamming=tag.hamming,
                decision_margin=tag.decision_margin,
                homography=tag.homography.flatten().astype(np.float32).tolist(),
                center=tag.center.tolist(),
                corners=tag.corners.flatten().tolist(),
                pose_error=tag.pose_err
            )
            # add detection to array
            msg.detections.append(detection)
            # publish tf
            self._tf_bcaster.sendTransform(
                p.tolist(),
                q.tolist(),
                detection_time,
                '/tag{:s}'.format(str(tag.tag_id)),
                self._camera_frame
            )
        # publish detections
        self._tag_pub.publish(msg)
        # update healthy frequency metadata
        self._tag_pub.set_healthy_freq(self._img_sub.get_frequency())
        self._img_pub.set_healthy_freq(self._img_sub.get_frequency())

    def _publish_detections_image(self, img, tags):
        # get a color buffer from the BW image
        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        # draw each tag
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0)
                )
            # draw the tag ID
            cv2.putText(
                color_img,
                str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255)
            )
        # pack image into a message
        img_msg = self._bridge.cv2_to_compressed_imgmsg(color_img)
        # ---
        self._img_pub.publish(img_msg)
        self._img_pub_busy = False

    def _cinfo_cb(self, data):
        K = np.array(data.K).reshape((3, 3))
        self._camera_parameters = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])
        self._camera_frame = data.header.frame_id
        # once we got the camera info, we can stop the subscriber
        self._cinfo_sub.shutdown()


def _matrix_to_quaternion(R):
    T = np.array((
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 1)
    ), dtype=np.float64)
    T[0:3, 0:3] = R
    return tf.transformations.quaternion_from_matrix(T)


if __name__ == '__main__':
    node = AprilTagDetector()