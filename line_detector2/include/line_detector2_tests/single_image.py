from collections import OrderedDict
from comptests.registrar import comptest, run_module_tests
import os

import cv2

from anti_instagram.AntiInstagram import AntiInstagram
from duckietown_utils.download import download_if_not_exist
from duckietown_utils.jpg import image_cv_from_jpg_fn, write_jpg_to_file
from duckietown_utils.path_utils import get_ros_package_path
from easy_algo.algo_db import get_easy_algo_db
from line_detector.visual_state_fancy_display import vs_fancy_display
from line_detector2.run_programmatically import FakeContext


@comptest
def single_image1():
    url = 'https://www.dropbox.com/s/bzezpw8ivlfu4b0/frame0002.jpg?dl=0'
    p = os.path.join(get_ros_package_path('line_detector2'), 
                     'include', 'line_detector2_tests', 'frame0002.jpg')
    download_if_not_exist(url, p)
    image_cv = image_cv_from_jpg_fn(p)
    
    line_detector_name = 'baseline'
    image_prep_name = 'prep_200_100'
    res  = run_pipeline(image_cv, line_detector_name, image_prep_name)
    write_images_as_jpegs('single_image1', res)
    
def write_images_as_jpegs(dirname, res):
    for i, (filename, image) in enumerate(res.items()):
        fn = os.path.join(dirname,('step%02d-'%i)+filename+'.jpg')
        write_jpg_to_file(image, fn)
        
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

    return res

if __name__ == '__main__':
    run_module_tests()