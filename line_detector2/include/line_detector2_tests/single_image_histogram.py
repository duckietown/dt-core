from collections import OrderedDict
from comptests.registrar import comptest, run_module_tests
import os

import cv2

from duckietown_utils.download import download_if_not_exist
from duckietown_utils.jpg import image_cv_from_jpg_fn
from duckietown_utils.path_utils import get_ros_package_path
from line_detector2_tests.single_image import write_images_as_jpegs
import numpy as np
from reprep.graphics.filter_posneg import posneg_hinton, posneg
from scipy import stats

@comptest
def single_image_histograms():
    url = 'https://www.dropbox.com/s/bzezpw8ivlfu4b0/frame0002.jpg?dl=0'
    p = os.path.join(get_ros_package_path('line_detector2'), 
                     'include', 'line_detector2_tests', 'frame0002.jpg')
    download_if_not_exist(url, p)
    image_cv = image_cv_from_jpg_fn(p)
    

    res  = go(image_cv)
    write_images_as_jpegs('single_image_histograms', res)
    
  

def go(image_bgr):
    res = OrderedDict()
    
    H, _W = image_bgr.shape[:2]
    cut = 0.3
    image_bgr_cut = image_bgr[int(cut*H):,:,:]
    
    res['image_bgr'] = image_bgr
    res['image_bgr_cut'] = image_bgr_cut
    
    hsv_map = np.zeros((180, 256, 3), np.uint8)
    hsv_map_h, hsv_map_s = np.indices(hsv_map.shape[:2])
    hsv_map[:,:,0] = hsv_map_h
    hsv_map[:,:,1] = hsv_map_s
    hsv_map[:,:,2] = 255
    hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)
#     cv2.imshow('hsv_map', hsv_map)
    res['hsv_map'] = hsv_map
    
    
    
    hist_scale = 10

    hsv = cv2.cvtColor(image_bgr_cut, cv2.COLOR_BGR2HSV)
#     dark = hsv[...,2] < 32
#     hsv[dark] = 0
    h0 = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])

    res['another'] = posneg(h0)
    
#     hf = h0.flatten()
#     c = np.empty_like(h0)
#     for i in range(c.shape[0]):
#         for j in range(c.shape[1]):
#             c[i,j]=stats.percentileofscore(hf, h0[i,j])
#     res['another2'] = posneg(c)
    
    h = h0 * hist_scale
#     h = np.clip(h*0.005*hist_scale, 0, 1)
    vis = hsv_map*h[:,:,np.newaxis] / 255.0
    res['vis'] = vis
    
    used = h> 0
    res['vis2'] = hsv_map * used[:,:,np.newaxis]
    return res

if __name__ == '__main__':
    run_module_tests() 