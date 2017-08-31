from comptests.registrar import comptest

from complete_image_pipeline.pipeline import run_pipeline
from duckietown_utils.image_writing import write_jpgs_to_dir
from duckietown_utils.jpg import image_cv_from_jpg_fn
from duckietown_utils.download import get_file_from_url

@comptest
def single_image1():
    url = 'https://www.dropbox.com/s/bzezpw8ivlfu4b0/frame0002.jpg?dl=0'
    p = get_file_from_url(url)
    image_cv = image_cv_from_jpg_fn(p)
    
    line_detector_name = 'baseline'
    image_prep_name = 'prep_200_100'
    res  = run_pipeline(image_cv, line_detector_name, image_prep_name)
    write_jpgs_to_dir(res, 'single_image1')
    
