import cv2

from duckietown_msgs.msg import Pixel, Vector2D  
import duckietown_utils as dtu
from geometry_msgs.msg import Point  
from image_geometry import PinholeCameraModel
import numpy as np
from sensor_msgs.msg import CameraInfo  


__all__ = [
    'GroundProjectionGeometry',
]

class GroundProjectionGeometry(object):
    
    """ 
    
        This class only knows about geometry, but not configuration files. 
        
        Conventions and coordinate frames:
        
            "vector"    Vector2D   (x, y)        normalized image coordinates
            "pixel"     Pixel      (u, v)        pixel coordinates
            "ground"    Point      (x, y, z=0)   axle frame
        
    """

    @dtu.contract(camera_info=CameraInfo, homography='array[3x3]')
    def __init__(self, camera_info, homography):
        self.ci = camera_info
        self.H = homography
        self.Hinv = np.linalg.inv(self.H)

        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(self.ci)
        
        # XXX: this needs to be removed
        self.rectified_input = False
        
    @dtu.contract(vec=Vector2D, returns=Pixel)
    def vector2pixel(self, vec):
        """ Converts a [0,1]*[0,1] representation to [0, W]x[0, H]. """
        pixel = Pixel()
        cw = self.ci.width
        ch = self.ci.height
        pixel.u = cw * vec.x
        pixel.v = ch * vec.y
        # XXX
        if False:
            # Nope, not this. Maybe set to NaN or throw an exception 
            if (pixel.u < 0): pixel.u = 0
            if (pixel.u > cw -1): pixel.u = cw - 1
            if (pixel.v < 0): pixel.v = 0
            if (pixel.v > ch - 1): pixel.v = 0
        return pixel

    @dtu.contract(pixel=Pixel, returns=Vector2D)
    def pixel2vector(self, pixel):
        """ Converts a [0,W]*[0,H] representation to [0, 1]x[0, 1]. """
        vec = Vector2D()
        vec.x = pixel.u / self.ci.width
        vec.y = pixel.v / self.ci.height
        return vec

    @dtu.contract(vec=Vector2D, returns=Point)
    def vector2ground(self, vec):
        """ Converts normalized coordinates to ground plane """
        pixel = self.vector2pixel(vec)
        return self.pixel2ground(pixel)

    @dtu.contract(point=Point, returns=Pixel) # NO
    def ground2vector(self, point):
        pixel = self.ground2pixel(point)
        return self.pixel2vector(pixel)

    @dtu.contract(pixel=Pixel, returns=Point)
    def pixel2ground(self, pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        if not self.rectified_input:
            uv_raw = self.pcm.rectifyPoint(uv_raw)
        #uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(self.H, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x / z
        point.y = y / z
        point.z = 0.0
        return point

    @dtu.contract(point=Point, returns=Pixel)
    def ground2pixel(self, point):
        if point.z != 0:
            msg = 'This method assumes that the point is a ground point (z=0). '
            msg +=  'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
            raise ValueError(msg)
            
        ground_point = np.array([point.x, point.y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]

        pixel = Pixel()
        if not self.rectified_input:
            dtu.logger.debug('project3dToPixel')
            distorted_pixel = self.pcm.project3dToPixel(image_point)
            pixel.u = distorted_pixel[0]
            pixel.v = distorted_pixel[1]
        else:
            pixel.u = image_point[0]
            pixel.v = image_point[1]
    
        return pixel
    
    def rectify_point(self, p):
        return self.pcm.rectifyPoint(p)

    def rectify(self, cv_image_raw):
        ''' Undistort image'''
        cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        W = self.pcm.width
        H = self.pcm.height
        mapx = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapy = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm.K, self.pcm.D, self.pcm.R, 
                                                 self.pcm.P, (W, H), 
                                                 cv2.CV_32FC1, mapx, mapy)
        return cv2.remap(cv_image_raw, mapx, mapy, cv2.INTER_CUBIC, cv_image_rectified)
