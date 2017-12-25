
import duckietown_utils as dtu
import numpy as np

from .map_localization_template import LocalizationTemplate
from geometry import translation_angle_from_SE2,\
    SE2_from_translation_angle

def phi_d_friendly(res):
    return 'phi: %s deg  d: %s m' % (np.rad2deg(res['phi']), res['d'])

class TemplateStraight(LocalizationTemplate):
    
    DATATYPE_COORDS = np.dtype([('phi', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_name):
        LocalizationTemplate.__init__(self, tile_name, TemplateStraight.DATATYPE_COORDS)
    
    def _init_metrics(self):
        if self._map is None:
            self.get_map() # need initialized
        width_yellow = self._map.constants['width_yellow']
        lane_width = self._map.constants['lane_width']
        self.offset = width_yellow / 2 + lane_width / 2
        
    @dtu.contract(returns='array', pose='SE2')
    def coords_from_pose(self, pose):
        """ Returns an array with datatype DATATYPE_COORDS """
        self._init_metrics()
        xy, theta = translation_angle_from_SE2(pose)
        
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['phi'] = dtu.norm_angle(theta)
        res['d'] = xy[1] + self.offset
        return res 
        
    @dtu.contract(returns='SE2', res='array')
    def pose_from_coords(self, res):
        self._init_metrics()
        x = 0
        y = res['d'] - self.offset
        theta = dtu.norm_angle(res['phi'])
        pose = SE2_from_translation_angle([x,y], theta)
        return pose



class TemplateBeforeCurve(LocalizationTemplate):
    
    DATATYPE_COORDS = np.dtype([('phi', 'float64'), ('d', 'float64')])
    
    def __init__(self, tile_name, direction):
        assert direction in ['left', 'right']
        self.direction = direction
        LocalizationTemplate.__init__(self, tile_name, TemplateStraight.DATATYPE_COORDS)
            
    def _init_metrics(self):
        if self._map is None:
            self.get_map() # need initialized
        width_yellow = self._map.constants['width_yellow']
        width_white = self._map.constants['width_white']
        lane_width = self._map.constants['lane_width']
        self.tile_size = self._map.constants['tile_size']
        self.offset = width_white + lane_width + width_yellow + lane_width / 2.0
        
        self.alpha0 = -np.deg2rad(45)

        self.center = np.array([+self.tile_size/2, +self.tile_size/2])
            
    @dtu.contract(returns='array', pose='SE2')
    def coords_from_pose(self, pose):
        """ Returns an array with datatype DATATYPE_COORDS """
        assert self.direction in ['left']
        
        self._init_metrics()
        
        xy, theta = translation_angle_from_SE2(pose)
        
        
        p = xy - self.center
        dist = np.hypot(p[0],p[1])
        d = self.offset - dist
#         dist = self.offset - d
        alpha = np.arctan2(p[1], p[0])
        
        forward =  alpha + np.pi/2
    
        phi = theta - forward
        
        res = np.zeros((), dtype=self.dt)
        # x is unused (projection) 
        res['phi'] = phi
        res['d'] = d
        return res 
        
    @dtu.contract(returns='SE2', res='array')
    def pose_from_coords(self, res):
        """ Returns an array with datatype dtu.DATATYPE_XYTHETA """
        
        assert self.direction in ['left']
        
        self._init_metrics()
        
        d = res['d']
        phi = res['phi']
        
        dist = self.offset - d
        forward = self.alpha0 + np.pi/2
        xy = self.center + dist * np.array([np.cos(self.alpha0), 
                                            np.sin(self.alpha0)])
        theta = forward + phi
        
        pose = SE2_from_translation_angle(xy, theta)
        return pose
