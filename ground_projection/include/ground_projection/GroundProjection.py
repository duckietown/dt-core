from duckietown_msgs import (Pixel, Vector2D)
from image_geometry import PinholeCameraModel
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
import os.path
from duckietown_utils import logger
from geometry_msgs import Point


class GroundProjection():
    def __init__(self):

        # defaults overwritten by param
        self.robot_name = "shamrock"
        self.rectified_input = False
        
        # read calibrations
        self.extrinsics_filename = (get_ros_package_path('duckietown') +
                               "/config/baseline/calibration/camera_extrinsic/" +
                               self.robot_name + ".yaml")
        if not os.path.isfile(extrinsics_filename):
            logger.warn("no robot specific extrinsic calibration, trying default")
            alternate_extrinsics_filename = (get_ros_package_path('duckietown') +
                                   "/config/baseline/calibration/camera_extrinsic/default.yaml")
            if not os.path.isfile(alternate_extrinsics_filename):
                logger.warn("can't find default either, something's wrong")
            else:
                self.H = self.load_homography(alternate_extrinsics_filename)
        else:
            self.H = self.load_homography(self.extrinsics_filename)
        self.H_inv = np.linalg.inv(self.H)
        
        intrinsics_filename = (get_ros_package_path('duckietown') + "/config/baseline/calibration/camera_intrinsics/" + self.robot_name + ".yaml")
        if not os.path.isfile(intrinsics_filename):
            logger.warn("no robot specific  calibration, trying default")
            intrinsics_filename = (get_ros_package_path('duckietown') +
                                   "/config/baseline/calibration/camera_extrinsic/default.yaml")
            if not os.path.isfile(extrinsics_filename):
                logger.error("can't find default either, something's wrong")

                
        self.ci_ = self.load_camera_info(intrinsics_filename)
        self.pcm_ = PinholeCameraModel()
        self.pcm_.fromCameraInfo(self.ci)

        
    def vector2pixel(self, vec):
        pixel = Pixel()
        cw = ci.width
        ch = ci.height
        pixel.u = cw*vec.x
        pixel.v = ch*vec.y
        if (pixel.u < 0) pixel.u =0
        if (pixel.u > cw -1) pixel.u = cw-1
        if (pixel.v < 0) pixel.v = 0
        if (pixel.v > ch - 1) pixel.v = 0
        return pixel

    def pixel2vector(self,pixel):
        vec = Vector2D()
        vec.x = pixel.u/ci.width
        vec.y = pixel.v/ci.height
        return vec

    def vector2ground(self, vec):
        pixel = self.vector2pixel(vec)
        return self.pixel2ground(pixel)

    def ground2vector(self,point):
        pixel = self.ground2image(point)
        return self.pixel2vector(pixel)

    def pixel2ground(self,pixel):
        uv_raw = np.array([pixel.u,pixel.v])
        if not self.rectified_input:
            uv_raw = self.pcm.rectifyPoint(uv_raw)
        uv_raw = [uv_raw,1]
        ground_point = self.H * uv_raw
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x/z
        point.y = y/z
        point.z = 0.0
        return point

    def ground2pixel(self,point):
        #TODO check whether z=0 or z=1.
        ground_point = np.array([point.x,point.y,1.0])
        image_point = self.Hinv * ground_point
        image_point /= image_point[2]

        pixel = Pixel()
        if not self.rectified_input:
            distorted_pixel = self.pcm.project3dToPixel(image_point)
            pixel.u = distorted_pixel[0]
            pixel.v = distorted_pixel[1]
        else:
            pixel.u = image_point[0]
            pixel.v = image_point[1]

            
            
    def rectify(self,cv_image_raw):
        cv_image_rectified = cvMat()
        self.pcm.rectifyImage(cv_image_raw, cv_image_rectified)
        return cv_image_rectified

    def estimate_homography(self,cv_image, board_size, offset):
        cv_image_rectified = self.rectify(cv_image)
        logger.info("image rectified")
        
        corners = cv2.findChessboardCorners(cv_image, board_size)
        if corners is None:
            logger.error("No corners found in image")
        criteria = (cv2.CV_TERMCRIT_EPS + cv2.CV_TERMCRIT_ITER,30,0.1)
        cv2.cornerSubPix(cv_image,corners,cv2.Size(11,11),cv2.Size(-1,-1),criteria)

        #TODO flip checks

        for r in range(board_h):
            for c in range(board_w):
                src_pts[r,c] = np.float32([r*square_size,c*square_size]) + offset

        self.H, mask = cv2.findHomography(src_pts, corners, method=cv2.CV_RANSAC)
        write_homography(self.extrinsic_filename)
    
    def load_homography(self, filename):
        data = yaml_load_file(filename)
        return np.array(data['homography']).reshape((3,3))

    def write_homography(self, filename):
        ob = {'Homography':self.H.resize(9,1)}
        yaml_write_to_file(ob,filename)
        
    def load_camera_info(self, filename):
        calib_data = yaml_load_file(filename)
        #     logger.info(yaml_dump(calib_data))
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3,3))
        cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1,5))
        cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3,3))
        cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def estimate_homography(self, 
