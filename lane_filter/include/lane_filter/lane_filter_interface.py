from abc import ABCMeta, abstractmethod


__all__ = [
    'LaneFilterInterface',
]


class LaneFilterInterface(object):

    __metaclass__ = ABCMeta
    
    LOST = 'lost'
    GOOD = 'good'
    STRUGGLING = 'struggling'
    
    POSSIBLE_STATUSES = [LOST, GOOD, STRUGGLING]
 
    @abstractmethod
    def initialize(self):
        pass
    
    @abstractmethod
    def predict(self, dt, v, w):
        pass
    
    @abstractmethod
    def update(self, segment_list):
        """
            segment list: a list of Segment objects
        """
    
    @abstractmethod
    def get_status(self):
        """ Returns one of the status above """
    
    # not sure exactly
#     @abstractmethod
#     def get_belief(self):
#         pass
    

    