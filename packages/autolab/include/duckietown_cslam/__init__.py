# coding=utf-8
__version__ = '0.1.0'

import logging

logging.basicConfig()
logger = logging.getLogger('dt-cslam')
logger.setLevel(logging.DEBUG)

logger.info('duckietown_cslam %s' % __version__)

