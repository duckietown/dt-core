from comptests.registrar import comptest

from complete_image_pipeline.pipeline import run_pipeline
from duckietown_utils.image_writing import write_jpgs_to_dir
from duckietown_utils.jpg import image_cv_from_jpg_fn
from duckietown_utils.download import get_file_from_url
from ground_projection import GroundProjection

@comptest
def single_image1():
    url = 'https://www.dropbox.com/s/bzezpw8ivlfu4b0/frame0002.jpg?dl=0'
    p = get_file_from_url(url)
    image_cv = image_cv_from_jpg_fn(p)
    
    line_detector_name = 'baseline'
    image_prep_name = 'baseline'
    lane_filter_name = 'baseline'
    robot_name = 'shamrock'
    gp = GroundProjection(robot_name)
#     gpg = gp.get_ground_projection_geometry()
    
    res,stats  = run_pipeline(image_cv, gp,
                         line_detector_name, image_prep_name, lane_filter_name,
                         all_details=False, skip_instagram=False, ground_truth=None)
    write_jpgs_to_dir(res, 'single_image1')
    
