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


def plot_reprojected_bgr(lane_filter, phi, d, segments, dpi=120):
    """ Returns a BGR image """  
    a = CreateImageFromPylab(dpi=dpi)

    f_d = lambda x: 100 * x
    f_phi = np.rad2deg
    lane_width = lane_filter.lanewidth
    d_max = lane_filter.d_max
    d_min = lane_filter.d_min
    phi_max = lane_filter.phi_max
    phi_min = lane_filter.phi_min
    linewidth_white = lane_filter.linewidth_white
    linewidth_yellow = lane_filter.linewidth_yellow
    horizon = lane_width * 3
    
    bounds = (f_d(-lane_width*3), f_d(lane_width*2), f_d(0), f_d(horizon))
    
    with a as pylab:
        def plot_line(y, *argv, **kwargs):
            pylab.plot([y, y], [0, f_d(horizon)], *argv, **kwargs)
            
        plot_line(f_d(lane_width/2), 'k-')
        plot_line(f_d(lane_width/2+linewidth_white), 'k-')
        plot_line(f_d(-lane_width/2), 'y--')
        plot_line(f_d(-lane_width/2-linewidth_yellow), 'y--')
        plot_line(f_d(-lane_width/2-linewidth_yellow-lane_width), 'k-')
        plot_line(f_d(-lane_width/2-linewidth_yellow-lane_width-linewidth_white), 'k-')
        # Where are the lanes?
        
        x = -d
        y = lane_width/3
        theta = phi + np.pi/2
        L = 0.10
        x1 = x + np.cos(theta)*L
        y1 = y + np.sin(theta)*L
        pylab.plot([f_d(x), f_d(x1)],[f_d(y),f_d(y1)], 'k')
        
        def transform(p):
            q = [p.x, p.y]
            x_ = x + q[0] * np.cos(theta) + q[1] * -np.sin(theta)
            y_ = y + q[0] * np.sin(theta) + q[1] * np.cos(theta)
            return [x_, y_]
    
        ax = pylab.gca()
        ax.patch.set_facecolor((0.5,0.5,0.5))
             
        for segment in segments:
            p0 = transform(segment.points[0])
            p1 = transform(segment.points[1])
            
            pylab.plot([f_d(p0[0]), f_d(p1[0])],[f_d(p0[1]),f_d(p1[1])], 'b-')
            
        
        pylab.axis(bounds)

    return a.get_bgr()