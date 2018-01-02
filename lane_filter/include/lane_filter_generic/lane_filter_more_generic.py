from collections import OrderedDict, defaultdict
from geometry import SE2_from_translation_angle, SO2_from_angle

from numpy.testing.utils import assert_almost_equal
from scipy.stats import entropy

from duckietown_segmaps.maps import get_normal_outward_for_segment, SegmentsMap
import duckietown_utils as dtu
from duckietown_utils.matplotlib_utils import CreateImageFromPylab
from easy_algo import get_easy_algo_db
from grid_helper import GridHelper, grid_helper_annotate_axes,\
    grid_helper_plot_field, grid_helper_mark_point, convert_unit,\
    grid_helper_set_axes
from lane_filter import LaneFilterInterface
from localization_templates import FAMILY_LOC_TEMPLATES
import numpy as np


__all__ = [
    'LaneFilterMoreGeneric',
]


class LaneFilterMoreGeneric(dtu.Configurable, LaneFilterInterface):
    """ """

    def __init__(self, configuration):
        param_names = [
            'localization_template',
            'delta_segment',   
            'min_max',
            'variables',
        ]

        dtu.Configurable.__init__(self, param_names, configuration)

        self.grid_helper = GridHelper(OrderedDict(self.variables))

        self.initialize_belief()
        self.last_segments_used = None
        
        self.delta_segment = float(self.delta_segment)
        
    def initialize_belief(self):
        self.belief = self.grid_helper.create_new()
        
        n = self.belief.shape[0]  * self.belief.shape[1]
        self.belief.fill(1.0/n)
        
        assert_almost_equal(self.belief.flatten().sum(), 1.0)
        
    def initialize(self):
        easy_algo_db = get_easy_algo_db()
        self._localization_template = \
            easy_algo_db.create_instance(FAMILY_LOC_TEMPLATES, 
                                         self.localization_template)
        self.initialize_belief()
        
    def predict(self, dt, v, w):
        pass

    def get_status(self):
        # TODO
        return LaneFilterInterface.GOOD

    def update(self, segment_list):
        """ Returns the likelihood """
        
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
        measurement_likelihood = self.grid_helper.create_new('float32')
        measurement_likelihood.fill(0)
        hit = miss = 0
        
        pose_weight = []
        
        num_by_color = defaultdict(lambda: 0)
        
        adjust_by_number = True # add to configuration
        
        with dtu.timeit_clock("pose gen for %s segs" % len(segments)):
            
            for segment in segments:
                num_by_color[segment.color] += 1
                
            for segment in segments:
                for pose, weight in self.generate_votes(segment, self.delta_segment):
                    
                    if adjust_by_number:
                        n = num_by_color[segment.color]
                        weight_adjusted = weight * 1.0 / n
                    else: 
                        weight_adjusted = weight
                    pose_weight.append((pose, weight_adjusted))
                   
        values = [] 
        with dtu.timeit_clock("voting for %s votes" % len(pose_weight)):
            for pose, weight in pose_weight:
                est = self._localization_template.coords_from_pose(pose)
                value = dict((k, est[k]) for k in self.variables)
                values.append(value)
        
        with dtu.timeit_clock("add voting for %s votes" % len(pose_weight)):
            for value in values:
                added = self.grid_helper.add_vote(measurement_likelihood, value, weight)
                hit += added > 0
                miss += added == 0
                
        dtu.logger.debug('hit: %s miss : %s' % (hit, miss))
        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        
        return measurement_likelihood

    def get_estimate(self):
        # TODO: make F parameter
        return self.grid_helper.get_max_weighted(self.belief, F=1)

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
                    pose = SE2_from_translation_angle(xy, theta)
                    yield pose, weight
                    num += 1
        if num == 0:
            msg = 'No segment found for %s' % segment.color
            dtu.logger.debug(msg)

    def get_plot_phi_d(self, ground_truth=None):
        a = CreateImageFromPylab(dpi=120)
        gh = self.grid_helper
        with a as pylab:
            grid_helper_plot_field(gh, self.belief, pylab)
            grid_helper_annotate_axes(gh, pylab)

            estimate = self.get_estimate()
            if ground_truth is not None:
                ground_truth_location = \
                    self._localization_template.coords_from_pose(ground_truth)
                grid_helper_mark_point(gh, pylab, ground_truth_location, 
                                       color='green', markersize=4)
            grid_helper_mark_point(gh, pylab, estimate, color='magenta', markersize=10)
            
            s = ''
            s += "status = %s" % self.get_status()
            for name, spec in zip(gh._names, gh._specs):
                convert = lambda x: '%.2f %s' % (convert_unit(x, spec.units, spec.units_display),
                                               spec.units_display) 
                s += '\n'
                s += "\nest %s = %s" % (name, convert(estimate[name]))
                if ground_truth is not None:
                    s += "\ntrue %s = %s" % (name, convert(ground_truth_location[name]))
                    err = np.abs(ground_truth_location[name] - estimate[name])
                    s += '\nabs err = %s' % (convert(err))
                    cell = spec.resolution
                    percent = 100.0 /cell *  err
                    s += '\nrel err = %.1f %% of cell' % (percent)
                    s += '\n true = green dot' 
                    
                    
            s += '\n'
            s += "\nentropy = %.4f" % self.get_entropy()
            s += "\nmax = %.4f" % self.belief.max()
            s += "\nmin = %.4f" % self.belief.min()

                
            pylab.annotate(s, xy=(0.7, 0.45), xycoords='figure fraction')
            grid_helper_set_axes(gh, pylab)
            
        return a.get_bgr()


@dtu.contract(sm=SegmentsMap, delta='float,>0')
def iterate_segment_sections(sm, map_segment, delta):
    """ Yields point, normal """
    w1 = np.array(sm.points[map_segment.points[0]].coords)
    w2 = np.array(sm.points[map_segment.points[1]].coords)
    dist = np.linalg.norm(w1-w2)
    
    if dist == 0:
        msg = 'Could not use degenerate segment (points: %s %s) ' % (w1, w2)
        raise ValueError(msg)
    
    map_segment_n = get_normal_outward_for_segment(w1, w2)
    # going from w1 to w2
    dirv = (w2-w1) / dist
    n = int(np.ceil(dist / delta))
    
    assert n >= 1
    for i in range(n):
        s = i + 0.5  # take middle of segment
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

