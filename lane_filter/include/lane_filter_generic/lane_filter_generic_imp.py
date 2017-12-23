from math import floor

from numpy.testing.utils import assert_almost_equal
from scipy.stats import entropy

from duckietown_segmaps.map_localization_template import FAMILY_LOC_TEMPLATES
from duckietown_segmaps.maps import get_normal_outward_for_segment, SegmentsMap
import duckietown_utils as dtu
from easy_algo.algo_db import get_easy_algo_db
from lane_filter import LaneFilterInterface
import numpy as np

from .visualization import plot_phi_d_diagram_bgr_generic
import itertools


__all__ = [
    'LaneFilterGeneric',
]

class LaneFilterGeneric(dtu.Configurable, LaneFilterInterface):
    """ """

    def __init__(self, configuration):
        param_names = [
            'localization_template',
            
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            
            'min_max',
        ]

        dtu.Configurable.__init__(self, param_names, configuration)

        self.d, self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,
                                   self.phi_min:self.phi_max:self.delta_phi]

        self.ref_d = np.empty_like(self.d)
        self.ref_phi = np.empty_like(self.phi)
        for i, j in itertools.product(range(self.d.shape[0]), range(self.d.shape[1])):
            self.ref_d[i,j] = self.d_min + (i+0.5)*self.delta_d
            self.ref_phi[i,j] = self.phi_min + (j+0.5)*self.delta_phi

        # these are the bounds you would give to pcolor
        # there is one row and one column more
        # self.d, self.phi are the lower corners

        # Each cell captures this area:
        #         (X[i,   j],   Y[i,   j]),
        #         (X[i,   j+1], Y[i,   j+1]),
        #         (X[i+1, j],   Y[i+1, j]),
        #         (X[i+1, j+1], Y[i+1, j+1])
        self.d_pcolor, self.phi_pcolor= \
            np.mgrid[self.d_min:(self.d_max+self.delta_d):self.delta_d,
                     self.phi_min:(self.phi_max+self.delta_phi):self.delta_phi]

        self.belief = np.empty(self.d.shape)

        self.last_segments_used = None
        

    def initialize(self):
        easy_algo_db = get_easy_algo_db()
        self._localization_template = easy_algo_db.create_instance(FAMILY_LOC_TEMPLATES, 
                                                                  self.localization_template)
        
        shape = self.d.shape
        n = shape[0] * shape[1]
        
        uniform = np.ones(dtype='float64',shape=self.d.shape) * (1.0/n)

        self.belief = uniform
        
        assert_almost_equal(self.belief.flatten().sum(), 1.0)

    def predict(self, dt, v, w):
        pass

    def get_status(self):
        return LaneFilterInterface.GOOD

    def update(self, segment_list):
        """ Returns the likelihood """
        
#         segment_list = fuzzy_segment_list_image_space(segment_list, n=100, intensity=0.0015)
        self.last_segments_used = segment_list.segments
        
        measurement_likelihood = self.generate_measurement_likelihood(segment_list.segments)
        if measurement_likelihood is not None:
            self.belief = np.multiply(self.belief, measurement_likelihood)
            if np.sum(self.belief) == 0:
                self.belief = measurement_likelihood
            
            self.belief = self.belief / np.sum(self.belief)

        return measurement_likelihood

    def generate_measurement_likelihood(self, segments):
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape, dtype='float32')
    
        hit = miss = 0
        for segment in segments:

            delta = 0.05
            for xytheta, weight in self.generate_votes(segment, delta): 
                
                est = self._localization_template.coords_from_xytheta(xytheta)

                d_i = est['d']
                phi_i = est['phi']

                # if the vote lands outside of the histogram discard it
                if d_i > self.d_max or \
                    d_i < self.d_min or \
                    phi_i < self.phi_min or \
                    phi_i > self.phi_max:
                    miss += 1
                    continue
                else:
                    hit += 1
                i = int(floor((d_i - self.d_min) / self.delta_d))
                j = int(floor((phi_i - self.phi_min) / self.delta_phi))
                
                m = 1.0
                K_d = lambda x: np.exp(-np.abs(x)*m/self.delta_d)
                K_phi = lambda x: np.exp(-np.abs(x)*m/self.delta_phi)
                F = 1
                
#                 F = 0
#                 K_phi = lambda _: 1
#                 K_d = lambda _: 1
#                 
                for di, dj in itertools.product(range(-F,F+1), range(-F,F+1)):
                    u = i + di
                    v = j + dj
                    if not ((0<=u<self.d.shape[0]) and (0<=v<self.d.shape[1])):
                        continue
                    phi_cell = self.ref_phi[u,v]
                    d_cell = self.ref_d[u,v]
                    delta_phi = phi_i - phi_cell
                    delta_d = d_i - d_cell
                    
                    k = K_phi(delta_phi) * K_d(delta_d)

                    measurement_likelihood[u, v] += weight * k
        
        dtu.logger.debug('hit: %s miss : %s' % (hit, miss))
        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        
        return measurement_likelihood

    def get_estimate(self):
        maxids = np.unravel_index(self.belief.argmax(), self.belief.shape)
        i, j = maxids
        d_max = self.ref_d[i,j]
        phi_max = self.ref_phi[i,j]
        res = np.zeros((), dtype=LaneFilterInterface.ESTIMATE_DATATYPE)
        res['d'] = d_max
        res['phi'] = phi_max
        return res

    @dtu.deprecated("use get_estimate")
    def getEstimate(self):
        """ Returns a list with two elements: (d, phi) """
        res = self.get_estimate()
        return [res['d'], res['phi']]

    def getMax(self):
        return self.belief.max()
    
    def get_entropy(self):
        s = entropy(self.belief.flatten())
        return s
    
    def generate_votes(self, segment, delta):
        """
            yields xytheta, weight
        """
        p1 = np.array([segment.points[0].x, segment.points[0].y, segment.points[0].z])
        p2 = np.array([segment.points[1].x, segment.points[1].y, segment.points[0].z])
        d = np.linalg.norm(p1-p2) 
        weight = d
        n_hat = get_normal_outward_for_segment(p2, p1)

        # SegmentsMap
        sm = self._localization_template.get_map()
        
        num = 0
        for map_segment in sm.segments:
            if map_segment.color == segment.color:
                for p, n in iterate_segment_sections(sm, map_segment, delta):
                    
                    xy, theta = get_estimate(p, n, p1, n_hat)
                    assert -np.pi <= theta <= +np.pi
                    
                    r = np.zeros(shape=(), dtype=dtu.DATATYPE_XYTHETA)
                    r['x'] = xy[0]
                    r['y'] = xy[1]
                    r['theta'] = theta 
                    yield r, weight
                    num += 1
        if num == 0:
            msg = 'No segment found for %s' % segment.color
            dtu.logger.debug(msg)

    def get_plot_phi_d(self, ground_truth=None):
        est = self.get_estimate()
        if ground_truth is not None:
            ground_truth_location = self._localization_template.coords_from_xytheta(ground_truth)
            phi_true = ground_truth_location['phi']
            d_true = ground_truth_location['d']
        return plot_phi_d_diagram_bgr_generic(self, phi=est['phi'], d=est['d'],
                                              phi_true=phi_true, d_true=d_true)

@dtu.contract(sm=SegmentsMap, delta='float,>0')
def iterate_segment_sections(sm, map_segment, delta):
    """ Yields point, normal """
    w1 = np.array(sm.points[map_segment.points[0]].coords)
    w2 = np.array(sm.points[map_segment.points[1]].coords)
    map_segment_n = get_normal_outward_for_segment(w1, w2)
    dirv = (w1-w2) / np.linalg.norm(w1-w2)
    n = int(np.floor(np.linalg.norm(w1-w2) / delta))
    for s in range(n):
        p = w1 + dirv * delta * s
        yield p, map_segment_n


def get_estimate(t, n, t_est, n_est):
    """ Returns xy, theta """
    # find theta that makes n and n_est rotate
    alpha1 = np.arctan2(n[1], n[0])
    alpha2 = np.arctan2(n_est[1], n_est[0])
    theta = dtu.norm_angle(alpha2 - alpha1)
    
    R = SO2_from_angle(theta)
    double_check = False
    if double_check:
#         print('alpha %s %s theta: %s' % (np.rad2deg(alpha1), 
#                                      np.rad2deg(alpha2), 
#                                      np.rad2deg(theta)))
        assert_almost_equal(np.dot(R, n[:2]), n_est[:2])
    
    t = t[0:2]
    t_est = t_est[0:2]
    xy = t - np.dot(R.T, t_est)
    
    
    # verify
    # xy + R(theta) t_est == t
    if double_check:
#         print('t %s t_est %s xy %s' % (t, t_est, xy))
        assert_almost_equal(xy + np.dot(R.T, t_est), t)

        
    return xy, -theta


def SO2_from_angle(theta):
    ''' Returns a 2x2 rotation matrix. '''
    return np.array([
            [+np.cos(theta), -np.sin(theta)],
            [+np.sin(theta), +np.cos(theta)]])
    