
from collections import OrderedDict
import os

import cv2

from duckietown_utils.bag_reading import d8n_bag_read_with_progress
from duckietown_utils.bag_visualization import d8n_make_video_from_bag
from duckietown_utils.bag_writing import d8n_write_to_bag_context
from duckietown_utils.cli import D8AppWithJobs
from duckietown_utils.exceptions import DTBadData
from duckietown_utils.image_conversions import d8n_image_msg_from_cv_image
from duckietown_utils.jpg import image_cv_from_jpg, make_images_grid
from duckietown_utils.system_cmd_imp import contract
from easy_algo.algo_db import get_easy_algo_db
from easy_logs.cli.easy_logs_summary_imp import format_logs
from easy_logs.logs_db import get_easy_logs_db
from line_detector.line_detector_interface import FAMILY_LINE_DETECTOR, LineDetectorInterface

from line_detector.visual_state import VisualState, visual_state_from_image
from duckietown_utils.image_rescaling import d8_image_zoom_linear
from line_detector.visual_state_fancy_display import vs_fancy_display


class RunLineDetectionTests(D8AppWithJobs): 
    """ Runs the line detection tests programmatically. """

    def define_options(self, params):
        params.add_string('algos', default=None,
            help="Line detectors query string. If not given, read from ALGOS") 
        params.add_string('logs', default=None,
            help="Log files query string. If not given, read from LOGS")
        params.add_string('dest', default=None,
            help='Output directory, where to write the ') 
        
    def define_jobs_context(self, context):
        # logger.setLevel(logging.DEBUG)
        query_logs = self.get_from_args_or_env('logs', 'LOGS')
        query_algos = self.get_from_args_or_env('algos', 'ALGOS')

        logs_db = get_easy_logs_db()
        
        dest = self.options.dest or 'out-detector-tests'
  
        logs = logs_db.query(query_logs, raise_if_no_matches=True)
        
        good_logs = get_only_valid_logs_with_camera(logs)
        
        if not good_logs:
            msg = 'Found no good logs to use.'
            raise DTBadData(msg)
        
        self.info('Found %d logs, of which %s good:\n%s' % 
                  (len(logs), len(good_logs), "\n".join(list(good_logs))))        
        
        self.debug('Log table: \n ' + format_logs(good_logs))
        
        # Find the algorithms
        algo_db = get_easy_algo_db()
        
        algos = algo_db.query(FAMILY_LINE_DETECTOR, query_algos, raise_if_no_matches=True)
        
        self.info('Found %d algos:\n%s' % (len(algos), "\n".join(list(algos))))
        
        do_videos = self.options.videos
        
        self.create_jobs(context, logs=good_logs, algos=algos, dest=dest, do_videos=do_videos) 
        
    def create_jobs(self, context, logs, algos, dest):
        for algo_name in algos:
            c = context.child(algo_name)
            for log_name, log in logs.items():
                if not log.valid:
                    msg = 'Skipping log %s because invalid' % log_name
                    self.warning(msg)
                out_bag = os.path.join(dest, log_name + '.bag')
                
                # Process the data 
                out_bag = c.comp(job, log_name, algo_name, out_bag, job_id=log_name)
                
                # Create the videos
                if do_videos:
                    for topic in ['processed']:
                        mp4 = os.path.join(dest, log_name + '-' + topic + '.mp4')
                        c.comp(d8n_make_video_from_bag, out_bag,  topic, mp4)
            
def job(log_name, algo_name, out_bag):
    logs_db = get_easy_logs_db()
    algo_db = get_easy_algo_db()
    log = logs_db.logs[log_name]
    line_detector = algo_db.create_instance(FAMILY_LINE_DETECTOR, algo_name)
    vehicle = log.vehicle    
    topic = '/%s/camera_node/image/compressed' % vehicle
    
    run_from_bag(log, topic, line_detector, out_bag)
    return out_bag


@contract(topic=str, line_detector=LineDetectorInterface)
def run_from_bag(log, topic, line_detector, out_bag_filename):
    H, W = 200, 320
    with d8n_write_to_bag_context(out_bag_filename) as out_bag:
        for image_msg in d8n_bag_read_with_progress(log, topic):
            # Get he CV image from a jpg
            image_cv = image_cv_from_jpg(image_msg.data)
            image_cv = cv2.resize(image_cv, (W, H), interpolation=cv2.INTER_NEAREST)
                
            vs = visual_state_from_image(image_cv, line_detector)
            
            rendered = vs_fancy_display(vs)
            rendered = d8_image_zoom_linear(rendered, 4)
            out = d8n_image_msg_from_cv_image(rendered, "bgr8", same_timestamp_as=image_msg)
            
            # Write to the bag
            out_bag.write('processed', out)
            out = d8n_image_msg_from_cv_image(image_cv, "bgr8", same_timestamp_as=image_msg)


def get_only_valid_logs_with_camera(logs):
    good_logs = OrderedDict()
    for k, log in logs.items():
        if not log.valid: 
            continue
        if not log.has_camera:
            continue
        good_logs[k] = log
    return good_logs
           
   
programmatic_line_detection_tests = RunLineDetectionTests.get_sys_main()

