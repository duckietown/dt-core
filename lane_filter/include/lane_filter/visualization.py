from duckietown_utils.matplotlib_utils import CreateImageFromPylab
import numpy as np
from reprep.graphics.filter_scale import scale

def plot_phi_d_diagram_bgr(lane_filter, phi, d, dpi=120):
    """ Returns a BGR image """  
    a = CreateImageFromPylab(dpi=dpi)
    with a as pylab:
        f_d = lambda x: 100 * x
        f_phi = np.rad2deg
        # Where are the lanes?
        lane_width = lane_filter.lanewidth
        d_max = lane_filter.d_max
        d_min = lane_filter.d_min
        phi_max = lane_filter.phi_max
        phi_min = lane_filter.phi_min
        delta_d = lane_filter.delta_d
        delta_phi = lane_filter.delta_phi
        

        W = f_d(lane_width/2)
        
        # note transpose
        belief_image = scale(-lane_filter.belief)
 
        x = f_d(lane_filter.d_pcolor)
        y = f_phi(lane_filter.phi_pcolor)  
        z = belief_image[:,:,0] # just R component
        
        pylab.pcolor(x, y, z, cmap='gray')
        
        pylab.plot(f_d(d), f_phi(phi), 'go', markersize=20, 
                   markeredgecolor='green',
                   markeredgewidth=3,
                   markerfacecolor='none')
        
        pylab.plot([-W,  -W], [f_phi(phi_min), f_phi(phi_max)], 'g--')
        pylab.plot([0, 0], [f_phi(phi_min), f_phi(phi_max)], 'g--')
        pylab.plot([+W,  +W], [f_phi(phi_min), f_phi(phi_max)], 'g--')

        
        pylab.axis([f_d(d_min), f_d(d_max), f_phi(phi_min), f_phi(phi_max)])
        pylab.ylabel('orientation (deg); cell = %.1f deg' % f_phi(delta_phi))
        pylab.xlabel('distance from center line (cm); cell = %.1f cm' % f_d(delta_d))

    return a.get_bgr()