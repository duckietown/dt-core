import numpy as np
import duckietown_utils as dtu
from localization_templates.map_localization_template import LocalizationTemplate
from geometry.poses import translation_angle_from_SE2,\
    SE2_from_translation_angle
from numpy.ma.testutils import assert_almost_equal

class TemplateXYStopline(LocalizationTemplate):
    """
    
        Coordinates:
        
            dstop: distance from stop line
            d: usual meaning
    
    
    """
    DATATYPE_COORDS_DSTOP = np.dtype([('dstop', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_name):
        LocalizationTemplate.__init__(self, tile_name, TemplateXYStopline.DATATYPE_COORDS_DSTOP)
    
        start = dict(d=0.04, dstop=0.24)
        pose = self.pose_from_coords(start)
        again = self.coords_from_pose(pose)
        assert_almost_equal(start['d'], again['d'])
        assert_almost_equal(start['dstop'], again['dstop'])
        
    def _init_metrics(self):
        if self._map is None:
            self.get_map() # need initialized
        width_yellow = self._map.constants['width_yellow']
        width_red = self._map.constants['width_red']
        tile_size = self._map.constants['tile_size']
        lane_width = self._map.constants['lane_width']
        self.offset = width_yellow / 2 + lane_width / 2
        
        stop_line_starts_at = tile_size/2 - width_red
        self.offset_dstop = stop_line_starts_at
        
        
    @dtu.contract(returns='array', pose='SE2')
    def coords_from_pose(self, pose):
        """ Returns an array with datatype DATATYPE_COORDS """
        self._init_metrics()
        xy, _ = translation_angle_from_SE2(pose)
        
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['d'] = xy[1] + self.offset # OK
        
        res['dstop'] = self.offset_dstop - xy[0]     
        return res 
        
    @dtu.contract(returns='SE2', res='array|dict')
    def pose_from_coords(self, res):
        self._init_metrics()
        
        y = res['d'] - self.offset # OK
        x = self.offset_dstop - res['dstop']

        theta = 0
        pose = SE2_from_translation_angle([x,y], theta)
        return pose
