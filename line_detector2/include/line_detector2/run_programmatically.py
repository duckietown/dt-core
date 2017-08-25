
from collections import OrderedDict
import logging
import os

import cv2
from quickapp.quick_app import QuickApp

from duckietown_utils import logger
from duckietown_utils.bag_reading import d8n_bag_read_with_progress
from duckietown_utils.bag_writing import d8n_write_to_bag_context
from duckietown_utils.exceptions import DTBadData, DTUserError
from duckietown_utils.image_conversions import d8n_image_msg_from_cv_image
from duckietown_utils.jpg import image_cv_from_jpg
from duckietown_utils.system_cmd_imp import contract
from easy_algo.algo_db import get_easy_algo_db
from easy_logs.cli.easy_logs_summary_imp import format_logs
from easy_logs.logs_db import get_easy_logs_db
from line_detector.line_detector_interface import FAMILY_LINE_DETECTOR, LineDetectorInterface
from line_detector.line_detector_plot import drawLines
import numpy as np


class RunLineDetectionTests(QuickApp): 

    def define_options(self, params):
        params.add_string('algos', default=None,
                          help="Line detectors query string. If not given, read from ALGOS") 
        params.add_string('logs', default=None,
                           help="Log files query string. If not given, read from LOGS")
        params.add_string('dest', default=None,
                          help='Output directory, where to write the ')
        
    def define_jobs_context(self, context):
        logger.setLevel(logging.DEBUG)
        logs_db = get_easy_logs_db()
        algo_db = get_easy_algo_db()
        
        query_logs = self.get_query_logs()
        query_algos = self.get_query_algos()
        dest = self.options.dest or 'out-detector-tests'
  
        logs = logs_db.query(query_logs)
        
        good_logs = get_only_valid_logs_with_camera(logs)
        
        if not good_logs:
            msg = 'Found no good logs to use.'
            raise DTBadData(msg)
        
        self.info('Found %d logs, of which %s good:\n%s' % 
                  (len(logs), len(good_logs), "\n".join(list(good_logs))))        
        
        self.debug('Log table: \n ' + format_logs(good_logs))
        
        algos = algo_db.query(FAMILY_LINE_DETECTOR, query_algos)
        
        if not algos:
            msg = 'Found no algos.'
            raise Exception(msg)
        
        self.info('Found %d algos:\n%s' % (len(algos), "\n".join(list(algos))))
        
        self.create_jobs(context, logs=good_logs, algos=algos, dest=dest)
        
    def create_jobs(self, context, logs, algos, dest):
        for algo_name in algos:
            c = context.child(algo_name)
            for log_name, log in logs.items():
                if not log.valid:
                    msg = 'Skipping log %s because invalid' % log_name
                    self.warning(msg)
                c.comp(job, log_name, algo_name, dest, job_id=log_name)

    def get_query_logs(self):
        options = [self.options.logs, os.environ.get('LOGS', None)]
        options = [_ for _ in options if _ and _.strip()]
        if not options:
            msg = 'Either provide command line argument --logs or environment variable LOGS.'
            raise DTUserError(msg)     
        return options[0]
    
    def get_query_algos(self):
        options = [self.options.algos, os.environ.get('ALGOS', None)]
        options = [_ for _ in options if _ and _.strip()]
        if not options:
            msg = 'Either provide command line argument --logs or environment variable ALGOS.'
            raise DTUserError(msg)     
        return options[0]

            
def get_only_valid_logs_with_camera(logs):
    good_logs = OrderedDict()
    for k, log in logs.items():
        if not log.valid: 
            continue
        if not log.has_camera:
            continue
        good_logs[k] = log
    return good_logs
        
def job(log_name, algo_name, dest):
    logs_db = get_easy_logs_db()
    algo_db = get_easy_algo_db()
    log = logs_db.logs[log_name]
    line_detector = algo_db.create_instance(FAMILY_LINE_DETECTOR, algo_name)

    vehicle = log.vehicle    
    topic = '/%s/camera_node/image/compressed' % vehicle
    out_bag = os.path.join(dest, 'out.bag')
    run_from_bag(log, topic, line_detector, out_bag)


@contract(topic=str, line_detector=LineDetectorInterface)
def run_from_bag(log, topic, line_detector, out_bag_filename):
    H, W = 200, 320
    with d8n_write_to_bag_context(out_bag_filename) as out_bag:
        for image_msg in d8n_bag_read_with_progress(log, topic):
            
            image_cv = image_cv_from_jpg(image_msg.data)
            image_cv = cv2.resize(image_cv, (W, H), interpolation=cv2.INTER_NEAREST)
                
            line_detector.setImage(image_cv)
            white = line_detector.detectLines('white')
            red = line_detector.detectLines('red')
            yellow = line_detector.detectLines('yellow')
            
            image_with_lines = np.copy(image_cv)
            drawLines(image_with_lines, white.lines, (0, 0, 0))
            drawLines(image_with_lines, yellow.lines, (255, 0, 0))
            drawLines(image_with_lines, red.lines, (0, 255, 0))
            
            out = d8n_image_msg_from_cv_image(image_cv, "bgr8", same_timestamp_as=image_msg)
            out_bag.write('processed', out)

   
   
programmatic_line_detection_tests = RunLineDetectionTests.get_sys_main()

