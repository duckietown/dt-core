import logging

from quickapp.quick_app import QuickApp

from duckietown_utils import logger


class RunLineDetectionTests(QuickApp): 

    def define_options(self, params):
        params.add_string('algos', help="Line detectors (comma separated)") 
        params.add_string('logs', help="bag files(comma separated)")
        params.add_string('dest', help='Output directory')
        
    def define_jobs_context(self, context):
        logger.setLevel(logging.DEBUG)
        algos = self.options.algos.split(',')
        logs = self.options.logs.split(',')

                
        for algo in algos:
            for log in logs:
                
        print 'hi', algos, logs


programmatic_line_detection_tests = RunLineDetectionTests.get_sys_main()

