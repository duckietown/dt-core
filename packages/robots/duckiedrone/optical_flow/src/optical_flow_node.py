#!/usr/bin/env python3

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage


class OpticalFlowNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OpticalFlowNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Initialize the parameters
        self.process_frequency = DTParam("~process_frequency", param_type=ParamType.INT)
        self.track_len = DTParam("~track_len", param_type=ParamType.INT)
        self.detect_interval = DTParam("~detect_interval", param_type=ParamType.INT)

        # obj attrs
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()

        # optical flow setup
        self.tracks = []
        self.frame_idx = 0
        self.prev_gray = None
        self._last_dx = 0.0
        self._last_dy = 0.0
        # todo: make param
        # params for ShiTomasi corner detection
        self.feature_params = dict(
            maxCorners=5,
            qualityLevel=0.5,
            minDistance=7,
            blockSize=7
        )
        # Parameters for lucas kanade optical flow
        self.lk_params = dict(
            winSize=(7, 7),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        # Subscriber
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cb_image, queue_size=1)

        # Publishers
        self.pub_debug_image = rospy.Publisher("~debug/image/compressed", CompressedImage, queue_size=1)
        self.pub_flow = rospy.Publisher("~flow", Vector3, queue_size=1)

        self.log("Initialization completed.")

    @staticmethod
    def draw_str(dst, target, s):
        x, y = target
        cv2.putText(dst, s, (x + 1, y + 1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)

    def cb_image(self, image_msg):
        # process at configured frequency
        now = rospy.Time.now()
        if now - self.last_stamp < rospy.Duration.from_sec(1.0 / self.process_frequency.value):
            return
        # if img msg too old, discard
        t_now = now.to_sec()
        t_stamp = image_msg.header.stamp.to_sec()
        if abs(t_now - t_stamp) > 0.3:
            return

        image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

        # # crop top 1.0 / 4
        # crop_ratio = 1.0 / 4
        # h_original = image.shape[0]
        # image = image[int(h_original * crop_ratio):, :, :]

        # resize by 1.0 / 4
        resize_scale = 1.0 / 4.0
        h_cropped, w_cropped = image.shape[:2]
        new_w_h = (int(resize_scale * w_cropped), int(resize_scale * h_cropped))
        image_cv = cv2.resize(image, new_w_h, interpolation=cv2.INTER_NEAREST)

        # should perform debug visualization?
        debug_viz_on = self.pub_debug_image.get_num_connections() > 0

        frame_gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        if debug_viz_on:
            vis = image_cv.copy()

        # if not done in the end, publish the same msg as last time
        has_published_flow = False
        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0 - p0r).reshape(-1, 2).max(-1)
            good = d < 1

            new_tracks = []
            speeds = []             # Unit: pixel / sec
            displacements = []      # Unit: pixel
            delta_t = float(t_now - self.last_stamp.to_sec())
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len.value:
                    del tr[1]
                new_tracks.append(tr)

                v_est = self.single_track_speed_est(track=tr, delta_t=delta_t)
                if v_est is not None:
                    speeds.append(v_est)

                est = self.single_track_est(track=tr)
                if est is not None:
                    displacements.append(est)

                if debug_viz_on:
                    cv2.circle(vis, (int(x), int(y)), 2, (0, 255, 0), -1)
            self.tracks = new_tracks

            debug_str = ""
            if len(speeds) > 0:
                speeds_arr = np.array(speeds)
                m_vx, m_vy = np.mean(speeds_arr, axis=0)
                std_v = np.std(speeds_arr, axis=0)
                debug_str = f"vx: {m_vx:>10.4f}, vy: {m_vy:>10.4f}, stddev: {std_v}\n"

            if len(displacements) > 0:
                disp_arr = np.array(displacements)
                m_x, m_y = np.mean(disp_arr, axis=0)
                std_xy = np.std(disp_arr, axis=0)
                debug_str += f"x: {m_x:>10.4f}, y: {m_y:>10.4f}, stddev: {std_xy}\n"
                self.publish_flow_msg(m_x, m_y)
                self._last_dx = m_x
                self._last_dx = m_y
                has_published_flow = True

            if debug_str != "":
                print(debug_str)

            if debug_viz_on:
                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                self.draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

            # dx, dy = self.single_track_est(0)
            # if dx is not np.NAN and dy is not np.NAN:
            #     self.publish_flow_msg(dx, dy)

        if self.frame_idx % self.detect_interval.value == 0 or len(self.tracks) == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **self.feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])

        self.frame_idx += 1
        self.prev_gray = frame_gray

        if debug_viz_on:
            self.pub_debug_image.publish(self.bridge.cv2_to_compressed_imgmsg(vis))

        if not has_published_flow:
            self.publish_flow_msg(0, 0)

        self.last_stamp = now


    def cb_pixels_to_meters(self, msg : Range):
    def cb_flow(self, flow_msg):
        # Convert velocity from pixels/s to m/s
        vx_m_s = flow_msg.x * self.pixels_to_meters.value
        vy_m_s = flow_msg.y * self.pixels_to_meters.value
        vz_m_s = 0  # Assuming z is also in pixels/s and needs conversion

        # Create a new Vector3 message for the converted velocity
        velocity_m_s_msg = Vector3()
        velocity_m_s_msg.x = vx_m_s
        velocity_m_s_msg.y = vy_m_s
        velocity_m_s_msg.z = vz_m_s

        # Publish the converted velocity
        self.pub_velocity_m_s.publish(velocity_m_s_msg)

        # Optionally, log the conversion for debugging
        rospy.loginfo(f"Converted velocity to m/s: ({vx_m_s}, {vy_m_s}, {vz_m_s})")

    def publish_flow_msg(self, dx: float, dy: float):
        flow_msg = Vector3()
        flow_msg.x = dx
        flow_msg.y = dy
        flow_msg.z = 0.0
        self.pub_flow.publish(flow_msg)

    @staticmethod
    def single_track_speed_est(track, delta_t):
        # speed est
        if len(track) > 1:
            x0, y0 = track[-2]
            x1, y1 = track[-1]
            vx = (x1 - x0) / delta_t
            vy = (y1 - y0) / delta_t
            return vx, vy
        else:
            return None

    @staticmethod
    def single_track_est(track, thres: float = 0.1):
        if len(track) > 1:
            x0, y0 = track[0]
            x1, y1 = track[-1]
            dx = x1 - x0
            dy = y1 - y0
            if abs(dx) < thres:
                dx = 0.0
            if abs(dy) < thres:
                dy = 0.0
            return dx, dy
        else:
            return None


if __name__ == "__main__":
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()
