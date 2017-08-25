import logging

from quickapp.quick_app import QuickApp

from duckietown_utils import logger
from easy_logs.logs_db import get_easy_logs_db
from easy_algo.algo_db import get_easy_algo_db
from easy_logs.scripts.easy_logs_summary_imp import format_logs


class RunLineDetectionTests(QuickApp): 

    def define_options(self, params):
        params.add_string('algos', help="Line detectors query string") 
        params.add_string('logs', help="Log files query string")
        params.add_string('dest', help='Output directory')
        
    def define_jobs_context(self, context):
        logger.setLevel(logging.DEBUG)
        logs_db = get_easy_logs_db()
        algo_db = get_easy_algo_db()
        self.info(type(logs_db.logs))
        logs = logs_db.query(self.options.logs)
        
        if not logs:
            msg = 'Found no logs.'
            raise Exception(msg)
        else:
            self.info('Found %d logs:\n%s' % (len(logs), "\n".join(list(logs))))        
        
        s = format_logs(logs)
        self.debug('Log table: \n ' + s)
        
        algos = algo_db.query(self.options.algos)
        
        if not algos:
            msg = 'Found no algos.'
            raise Exception(msg)
        
        self.info('Found %d algos:\n%s' % (len(algos), "\n".join(list(algos))))
        
                  


programmatic_line_detection_tests = RunLineDetectionTests.get_sys_main()

