from abc import ABCMeta, abstractmethod

import duckietown_utils as dtu

__all__ = [
    'ProcessorInterface',
    'ProcessorUtilsInterface',
]


class ProcessorUtilsInterface(object):

    __metaclass__ = ABCMeta

    @abstractmethod
    def write_stat(self, t, name, value):
        pass

    @abstractmethod
    def get_log(self):
        pass


class ProcessorInterface(object):

    FAMILY = 'processor'

    __metaclass__ = ABCMeta

    @abstractmethod
    @dtu.contract(utils=ProcessorUtilsInterface)
    def process_log(self, bag_in, prefix, bag_out, utils):
        pass

