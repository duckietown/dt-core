
from duckietown_segmaps import FAMILY_SEGMAPS, SegmentsMap
import duckietown_utils as dtu
from easy_algo import get_easy_algo_db
import numpy as np

from .map_localization_template import LocalizationTemplate
from duckietown_segmaps.maps import FRAME_GLOBAL


class TemplateStraight(LocalizationTemplate):
    
    DATATYPE_COORDS = np.dtype([('phi', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_name):
        self.tile_name = tile_name
        self.dt = TemplateStraight.DATATYPE_COORDS
        self._map = None
        
    @dtu.contract(returns=SegmentsMap)
    def get_map(self):
        if self._map is None:
            # Need to do this here and not in constructor
            # because otherwise there is a loop in EasyAlgo
            db = get_easy_algo_db()
            self._map = db.create_instance(FAMILY_SEGMAPS, self.tile_name)
            width_yellow = self._map.constants['width_yellow']
            lane_width = self._map.constants['lane_width']
            self.offset = width_yellow / 2 + lane_width / 2
            
            frames = set(_.id_frame for _ in self._map.points.values())
            if frames != set([FRAME_GLOBAL]):
                msg = ('Expected that all points in the map %r are in the frame %r.' % 
                       (self.tile_name, FRAME_GLOBAL))
                msg += ' These are the frames: %s.' % frames
                raise ValueError(msg)
        return self._map
    
    def get_coords_datatype(self):
        return self.dt 
    
    @dtu.contract(returns='array', xytheta='array')
    def coords_from_xytheta(self, xytheta):
        """ Returns an array with datatype DATATYPE_COORDS """
        
        self.get_map() # need initialized
        
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['phi'] = dtu.norm_angle(xytheta['theta'])
        res['d'] = xytheta['y'] + self.offset
        return res 
        
    @dtu.contract(returns='array', res='array')
    def xytheta_from_coords(self, res):
        """ Returns an array with datatype dtu.DATATYPE_XYTHETA """
        self.get_map() # need initialized
        
        r = np.zeros((), dtype=dtu.DATATYPE_XYTHETA)
        r['x'] = 0
        r['y'] = res['d'] - self.offset
        r['theta'] = dtu.norm_angle(res['phi'])
        return r

