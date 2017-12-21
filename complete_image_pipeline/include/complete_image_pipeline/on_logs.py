from quickapp import QuickApp

import duckietown_utils as dtu
from duckietown_utils.cli import D8AppWithLogs
from ground_projection import GroundProjection
import rosbag 

from .pipeline import run_pipeline
import os


__all__ = [
    'SingleImagePipelineLog',
]

class SingleImagePipelineLog(D8AppWithLogs, QuickApp):
    """ 
        Runs the vision pipeline on the images in a log.  
    """

    def define_options(self, params):
        g = "Input/output"
        params.add_string('log', help="Log to use.", group=g)
#         params.add_string('output', default=None, short='-o', help='Output directory', group=g) 
        g = "Pipeline"
        params.add_string('line_detector', default='baseline', 
                          help="Which line detector to use", group=g)
        params.add_string('image_prep', default='prep_200_100', 
                          help="Which image prep to use", group=g)
        params.add_string('lane_filter', default='baseline',
                          help="Which lane filter to use", group=g)
        
    def define_jobs_context(self, context):
        db = self.get_easy_logs_db() 
        query = self.options.log
        line_detector = self.options.line_detector
        image_prep = self.options.image_prep
        lane_filter = self.options.lane_filter
    
        logs = db.query(query)
     
        for k, log in logs.items():
            d = os.path.join(self.options.output, k)
            context.comp(look_at, log, d,
                         line_detector, image_prep, lane_filter)

def look_at(log, output, line_detector, image_prep, lane_filter):
    filename = log.filename
    
    bag = rosbag.Bag(filename) 
    
    vehicle_name = dtu.which_robot(bag)
    
    dtu.logger.info('Vehicle name: %s' % vehicle_name) 
    
    gp = GroundProjection(vehicle_name) 

    topic = dtu.get_image_topic(bag)
    res = dtu.d8n_read_all_images_from_bag(bag, topic, max_images=1)
    
    image_cv = res[0]['rgb']
    
#     dtu.logger.debug(dtu.describe_value(image_cv))

    image_cv_bgr = dtu.bgr_from_rgb(image_cv)
    res  = run_pipeline(image_cv_bgr, gp=gp,
                        line_detector_name=line_detector, 
                        image_prep_name=image_prep,
                        lane_filter_name=lane_filter)

    res = dtu.resize_small_images(res)
         
    dtu.write_bgr_images_as_jpgs(res, output)
    
