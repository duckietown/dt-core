from collections import OrderedDict
import copy

import cv2

from anti_instagram.AntiInstagram import AntiInstagram
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import yaml_load_file
from easy_algo.algo_db import get_easy_algo_db
from line_detector.visual_state_fancy_display import vs_fancy_display
from line_detector2.run_programmatically import FakeContext
import numpy as np
from sensor_msgs.msg import CameraInfo  # @UnresolvedImport
from comptests.registrar import run_module_tests



def run_pipeline(image, line_detector_name, image_prep_name):
    """ 
        Image: numpy (H,W,3) == BGR
        Returns a dictionary, res with the following fields:
            
            res['input_image']
    """
    res = OrderedDict()
    res['image_input'] = image
    algo_db = get_easy_algo_db()
    line_detector = algo_db.create_instance('line_detector', line_detector_name)
    image_prep = algo_db.create_instance('image_prep', image_prep_name)

    context = FakeContext()
    
    segment_list = image_prep.process(context, image, line_detector, transform = None)

    res['segments_on_image_input'] = vs_fancy_display(image_prep.image_cv, segment_list)    
    res['segments_on_image_resized'] = vs_fancy_display(image_prep.image_resized, segment_list)
    
    ai = AntiInstagram()
    ai.calculateTransform(res['image_input'])
    
    transform = ai.applyTransform 
    
    res['image_input_transformed'] = transform(res['image_input'])
    res['image_input_transformed_then_convertScaleAbs'] = cv2.convertScaleAbs(res['image_input_transformed'])

    segment_list2 = image_prep.process(context, image, line_detector, transform=transform)
    
    res['segments2_on_image_input_transformed'] = \
        vs_fancy_display(image_prep.image_cv, segment_list2)    
    res['segments2_on_image_input_transformed_resized'] = \
        vs_fancy_display(image_prep.image_resized, segment_list2)

    filename = (get_ros_package_path('duckietown') + 
                "/config/baseline/calibration/camera_intrinsic/default.yaml")
                            
    ci = load_camera_info(filename)
#     (h,w) = image.shape[:2]
    ci_W = ci.width
    ci_H = ci.height
    mapx, mapy = cv2.initUndistortRectifyMap(ci.K, ci.D, ci.R, ci.P, (ci_W,ci_H), cv2.CV_32FC1)
    # mapx and mapy are (h, w) matrices that tell you 
    # the x coordinate and the y coordinate for each point 
    # in the first image
#     print mapx.shape(),mapy.shape()
    res['grid'] = get_grid(image.shape[:2])
    res['grid_remap'] = cv2.remap(res['grid'], mapx, mapy, cv2.INTER_LINEAR)
    
    res['image_input_rect'] = cv2.remap(res['image_input'], mapx, mapy, cv2.INTER_LINEAR)
    segment_list_rect = apply_transform_to_segment(segment_list, mapx, mapy)
    res['segments2_on_image_input_rect'] = \
        vs_fancy_display(res['image_input_rect'], segment_list_rect)

    res['grid_undistort'] = cv2.undistort(res['grid'], ci.K, ci.D)
    res['image_input_undistort'] = cv2.undistort(res['image_input'], ci.K, ci.D)
    
    homography =np.array( [-4.89775e-05, -0.0002150858, -0.1818273, 
                           0.00099274, 1.202336e-06, -0.3280241,
                            -0.0004281805, -0.007185673, 1]).reshape((3,3))
    
    segment_list_undistort = undistort_segments(segment_list, ci_W, ci_H, ci.K, ci.D, ci.P, ci.R, homography=homography)
    res['segments_und_image_input_und'] = vs_fancy_display(res['image_input_undistort'], segment_list_undistort)
    
                            
#     homography_inv = np.linalg.inv(homography)
    res['image_input_undistort_warp'] = cv2.warpPerspective(res['image_input'], homography,
                                                             (ci_W,ci_H),flags=cv2.WARP_INVERSE_MAP)
    # add flags=cv2.WARP_INVERSE_MAP to do the inverse
    
    
    return res

def get_grid(shape, L=32):
    H, W = shape
    res = np.zeros((H,W,3),'uint8')
    for i in range(H):
        for j in range(W):
            cx = int(i/L) 
            cy = int(j/L)
            coli = (cx + cy) %2
            col = {0: (255,0,0), 1: (0,255,0)}
            res[i,j,:] = col[coli]
    return res

def undistort_segments(segment_list, ci_W, ci_H, K, D, P, R, homography):
    segment_list = copy.deepcopy(segment_list)
    for segment in segment_list.segments:
        for i in range(2):
            p = segment.pixels_normalized[i]
            assert 0 <= p.x <= 1
            assert 0 <= p.y <= 1
            x0 = p.x * ci_W
            y0 = p.y * ci_H
            x1, y1 = undistort(x0, y0, K, D, P, R)
            x1 = (x1*1.0)/ci_W
            y1 = (y1*1.0)/ci_H
            #print('%s %s -> %s %s' % (p.x,p.y,x1,y1))
            p.x=x1
            p.y=y1
            # These are real space
            # XXX these are certainly wrong
#             r = np.array([p.x, p.y, 1])
#             r2 = np.dot(np.linalg.inv(homography), r)
#             print r2
#             W = segment.points[i]
#             W.x = r2[0] / r2[2]
#             W.y = r2[1] / r2[2]
#             W.z = 0
#             print W
#             
            
    return segment_list

def undistort(x, y, K, D, P, R):
    src = np.zeros((1,1,2))  
    src[0,0,0] = x
    src[0,0,1] = y
    dst = cv2.undistortPoints(src, cameraMatrix=K, distCoeffs=D,
                              P=P, 
                              R=R) 
    x = dst[0,0,0]
    y = dst[0,0,1]
    return x,y
    
def apply_transform_to_segment(segment_list, mapx, mapy):
    segment_list = copy.deepcopy(segment_list)
    for segment in segment_list.segments:
        transform(segment.pixels_normalized[0], mapx, mapy)
        transform(segment.pixels_normalized[1], mapx, mapy)
    return segment_list

        
def transform(p, mapx, mapy):
    """ Transform in place the point (p.x,p.y) """
    H, W = mapx.shape
    assert 0 <= p.x <= 1
    assert 0 <= p.y <= 1
    u = int(np.round(p.x * W))
    v = int(np.round(p.y * H))
    # get new points
    p.x = mapx[v,u] / W
    p.y = mapy[v,u] / H
    
def load_camera_info(filename):
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


if __name__ == '__main__':
    run_module_tests()
