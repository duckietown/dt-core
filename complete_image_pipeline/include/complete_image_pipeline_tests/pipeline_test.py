from complete_image_pipeline.pipeline import run_pipeline
from ground_projection import GroundProjection

import duckietown_utils as dtu

@dtu.unit_test
def single_image1():
    url = 'https://www.dropbox.com/s/bzezpw8ivlfu4b0/frame0002.jpg?dl=0'
    p = dtu.get_file_from_url(url)
    image_cv = dtu.image_cv_from_jpg_fn(p)
    
    line_detector_name = 'baseline'
    image_prep_name = 'baseline'
    lane_filter_name = 'baseline'
    robot_name = 'shamrock'
    gp = GroundProjection(robot_name)
#     gpg = gp.get_ground_projection_geometry()
    
    res, _stats  = run_pipeline(image_cv, gp,
                         line_detector_name, image_prep_name, lane_filter_name,
                         all_details=False, skip_instagram=False, ground_truth=None)
    
    outd = dtu.get_output_dir_for_test()
    dtu.write_jpgs_to_dir(res, outd)
    
