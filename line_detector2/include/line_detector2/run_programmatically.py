
from contextlib import contextmanager

from duckietown_utils.bag_info import which_robot
from duckietown_utils.bag_reading import d8n_bag_read_with_progress
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.image_conversions import d8n_image_msg_from_cv_image
from duckietown_utils.image_rescaling import d8_image_zoom_linear
from duckietown_utils.image_timestamps import add_duckietown_header
from duckietown_utils.jpg import image_cv_from_jpg
from duckietown_utils.system_cmd_imp import contract
from easy_algo.algo_db import get_easy_algo_db
from easy_regression.processor_interface import ProcessorInterface
from line_detector.line_detector_interface import FAMILY_LINE_DETECTOR
from line_detector.visual_state_fancy_display import vs_fancy_display


class LineDetectorProcessor(ProcessorInterface):
    
    @contract(image_prep='str', line_detector='str')
    def __init__(self, image_prep, line_detector):
        self.image_prep = image_prep
        self.line_detector = line_detector
        
    def process_log(self, bag_in, bag_out):
        algo_db = get_easy_algo_db()
        line_detector = algo_db.create_instance(FAMILY_LINE_DETECTOR, self.line_detector)
        image_prep = algo_db.create_instance('image_prep', self.image_prep)
        
        vehicle = which_robot(bag_in)
        topic = '/%s/camera_node/image/compressed' % vehicle
        context = FakeContext()
        transform = None
        frame = 0
        for compressed_img_msg in d8n_bag_read_with_progress(bag_in, topic):
            
            with context.phase('decoding'):
                try:
                    image_cv = image_cv_from_jpg(compressed_img_msg.data)
                except ValueError as e:
                    msg = 'Could not decode image: %s' % e
                    raise_wrapped(ValueError, e, msg)
                
            segment_list = image_prep.process(context, image_cv, line_detector, transform)
            
            rendered = vs_fancy_display(image_prep.image_cv, segment_list)
            rendered = d8_image_zoom_linear(rendered, 2)
            log_name = 'log_name'
            time = 12
            rendered = add_duckietown_header(rendered, log_name, time, frame)
            out = d8n_image_msg_from_cv_image(rendered, "bgr8", same_timestamp_as=compressed_img_msg)
            
            
            # Write to the bag
            bag_out.write('processed', out)
            
#             out = d8n_image_msg_from_cv_image(image_cv, "bgr8", same_timestamp_as=compressed_img_msg)
            bag_out.write('image', compressed_img_msg)
            
            frame+=1
     

class FakeContext():
    def __init__(self):
        pass  

    @contextmanager
    def phase(self, name):  # @UnusedVariable
            yield

    def get_stats(self):
        pass 
