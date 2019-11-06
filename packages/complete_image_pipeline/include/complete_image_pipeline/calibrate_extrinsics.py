from collections import OrderedDict
import os

import cv2

import duckietown_utils as dtu
from duckietown_utils.cli import D8App
from ground_projection.configuration import get_homography_default, \
    disable_old_homography
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
from ground_projection.ground_projection_interface import estimate_homography, \
     HomographyEstimationResult, save_homography
from pi_camera.camera_info import get_camera_info_for_robot

__all__ = [
    'CalibrateExtrinsics',
]


class CalibrateExtrinsics(D8App):
    """
        Calibrate the extrinsics. Run on Duckiebot directly.
    """

    cmd = 'rosrun complete_image_pipeline calibrate_extrinsics'

    def define_program_options(self, params):
        g = "Input/output"
        params.add_string('input', default=None, help='If given, use this image rather than capturing.')
        params.add_string('output', default=None, short='-o', help='Output directory', group=g)

    def go(self):
        robot_name = dtu.get_current_robot_name()

        output = self.options.output
        if output is None:
            output = 'out-calibrate-extrinsics'  #  + dtu.get_md5(self.options.image)[:6]
            self.info('No --output given, using %s' % output)

        if self.options.input is None:

            print("{}\nCalibrating using the ROS image stream...\n".format("*"*20))
            import rospy
            from sensor_msgs.msg import CompressedImage

            topic_name = os.path.join('/', robot_name, 'camera_node/image/compressed')
            print('Topic to listen to is: %s' % topic_name)

            print('Let\'s wait for an image. Say cheese!')
            img_msg = None

            rospy.init_node('test')

            try:
                img_msg = rospy.wait_for_message(topic_name, CompressedImage, timeout=10)
                print('Image captured!')

            except rospy.ROSException as e:
                print('Didn\'t get any message!: %s' % (e,))

            bgr = dtu.bgr_from_rgb(dtu.rgb_from_ros(img_msg))
            self.info('Picture taken: %s ' % str(bgr.shape))

        else:
            self.info('Loading input image %s' % self.options.input)
            bgr = dtu.bgr_from_jpg_fn(self.options.input)

        if True:
            interpolation = cv2.INTER_CUBIC
            bgr = dtu.d8_image_resize_fit(bgr, 640, interpolation)
            self.info('Resized to: %s ' % str(bgr.shape))

        robot_name = dtu.get_current_robot_name()
        disable_old_homography(robot_name)

        camera_info = get_camera_info_for_robot(robot_name)
        homography_dummy = get_homography_default()
        gpg = GroundProjectionGeometry(camera_info, homography_dummy)

        res = OrderedDict()
        try:
            bgr_rectified = gpg.rectify(bgr, interpolation=cv2.INTER_CUBIC)

            res['bgr'] = bgr
#            if False:
#                bgr = equalize(bgr)
#                res['equalized'] = bgr
            res['bgr_rectified'] = bgr_rectified

            if True:
#                _, res['rectified_full'] = gpg.rectify_full(bgr)
                _new_matrix, res['rectified_full_ratio_auto'] = gpg.rectify_full(bgr, ratio=1.65)
            result = estimate_homography(bgr_rectified)
            dtu.check_isinstance(result, HomographyEstimationResult)
#
#            if result.bgr_detected is not None:
#                res['bgr_detected'] = result.bgr_detected

            if result.bgr_detected_refined is not None:
                res['bgr_detected_refined'] = result.bgr_detected_refined

            if not result.success:
                raise Exception(result.error)

            save_homography(result.H, robot_name)

            msg = '''

To check that this worked well, place the robot on the road, and run:

    rosrun complete_image_pipeline single_image

Look at the produced jpgs.

'''
            self.info(msg)
        finally:
            dtu.write_bgr_images_as_jpgs(res, output)

#
#def equalize(bgr):
#    img_yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV)
#
#    # equalize the histogram of the Y channel
#    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
#
#    # convert the YUV image back to RGB format
#    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
#    return img_output
