from collections import OrderedDict
from math import floor

from numpy.testing.utils import assert_almost_equal
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal, entropy

from duckietown_msgs.msg import SegmentList
import duckietown_utils as dtu
import numpy as np

from .lane_filter_interface import LaneFilterInterface
from .visualization import plot_phi_d_diagram_bgr

__all__ = [
    'LaneFilterHistogram',
]


class LaneFilterHistogram(dtu.Configurable, LaneFilterInterface):
    '''
        The one developed in Fall 2017 by the Controllers
    '''

    def __init__(self, configuration):
        param_names = [
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'cov_v',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'sigma_d_mask',
            'sigma_phi_mask',
        ]
        dtu.Configurable.__init__(self, param_names, configuration)

        self.d, self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,
                                   self.phi_min:self.phi_max:self.delta_phi]
        # these are the bounds you would give to pcolor
        # there is one row and one column more
        # self.d, self.phi are the lower corners

        # Each cell captures this area:
        #         (X[i,   j],   Y[i,   j]),
        #         (X[i,   j+1], Y[i,   j+1]),
        #         (X[i+1, j],   Y[i+1, j]),
        #         (X[i+1, j+1], Y[i+1, j+1])
        self.d_pcolor, self.phi_pcolor = \
            np.mgrid[self.d_min:(self.d_max + self.delta_d):self.delta_d,
                     self.phi_min:(self.phi_max + self.delta_phi):self.delta_phi]

        num_belief = 4
        self.num_belief = num_belief

        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [ [self.sigma_d_0, 0], [0, self.sigma_phi_0] ]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        self.last_segments_used = None

        range_arr = np.zeros(num_belief + 1)
        range_max = 0.6  # range to consider edges in general
        range_min = 0.2
        range_diff = (range_max - range_min) / (num_belief - 1)

        for i in range(1, num_belief + 1):
            range_arr[i] = range_min + (i - 1) * range_diff

        self.range_arr = range_arr

        self.initialize()

#
    def initialize(self):
        self.beliefArray = []
        for _ in range(self.num_belief):
            n = self.d.shape[0] * self.d.shape[1]
            b = np.ones(self.d.shape) * (1.0 / n)
            self.beliefArray.append(b)

        pos = np.empty(self.d.shape + (2,))
        pos[:, :, 0] = self.d
        pos[:, :, 1] = self.phi
        # XXX: statement with no effect
        # self.cov_0
        RV = multivariate_normal(self.mean_0, self.cov_0)

        n = pos.shape[0] * pos.shape[1]

        gaussian = RV.pdf(pos) * 0.5  #+ 0.5/n

        gaussian = gaussian / np.sum(gaussian.flatten())

        uniform = np.ones(dtype='float32', shape=self.d.shape) * (1.0 / n)

        a = 0.01
        self.belief = a * gaussian + (1 - a) * uniform

        assert_almost_equal(self.belief.flatten().sum(), 1.0)

    def get_status(self):
        # TODO: Detect abnormal states (@liam)
        return LaneFilterInterface.GOOD

    def predict(self, dt, v, w):
        delta_t = dt
        d_t = self.d + v * delta_t * np.sin(self.phi)
        phi_t = self.phi + w * delta_t

        for k in range(self.num_belief):
            p_belief = np.zeros(self.beliefArray[k].shape)

            # there has got to be a better/cleaner way to do this - just applying the process model to translate each cell value
            for i in range(self.beliefArray[k].shape[0]):
                for j in range(self.beliefArray[k].shape[1]):
                    if self.beliefArray[k][i, j] > 0:
                        if d_t[i, j] > self.d_max or d_t[i, j] < self.d_min or phi_t[i, j] < self.phi_min or phi_t[i, j] > self.phi_max:
                            continue
                        i_new = int(floor((d_t[i, j] - self.d_min) / self.delta_d))
                        j_new = int(floor((phi_t[i, j] - self.phi_min) / self.delta_phi))
                        p_belief[i_new, j_new] += self.beliefArray[k][i, j]

            s_belief = np.zeros(self.beliefArray[k].shape)
            gaussian_filter(p_belief, self.cov_mask, output=s_belief, mode='constant')

            if np.sum(s_belief) == 0:
                return
            self.beliefArray[k] = s_belief / np.sum(s_belief)

    def update(self, segments):
        range_arr = self.range_arr
        for i in range(self.num_belief):
            if i == 0:
                measurement_likelihood = self.generate_measurement_likelihood(segments, range_arr[i], range_arr[i + 2])
            else:
                measurement_likelihood = self.generate_measurement_likelihood(segments, range_arr[i], range_arr[i + 1])

            if measurement_likelihood is not None:
                self.beliefArray[i] = np.multiply(self.beliefArray[i], measurement_likelihood)
                if np.sum(self.beliefArray[i]) == 0:
                    self.beliefArray[i] = measurement_likelihood
                else:
                    self.beliefArray[i] = self.beliefArray[i] / np.sum(self.beliefArray[i])
        return measurement_likelihood

    @dtu.contract(segment_list=SegmentList)
    def generate_measurement_likelihood(self, segment_list, range_min, range_max):
        segments = segment_list
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape, dtype='float32')
        for segment in segments:
            # only consider points in a certain range from the Duckiebot
            point_range = self.getSegmentDistance(segment)
            if point_range < range_min or point_range > range_max:
                continue
            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            d_i, phi_i, _l_i, weight = self.generateVote(segment)
            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or \
                d_i < self.d_min or \
                phi_i < self.phi_min or \
                phi_i > self.phi_max:
                continue
            i = int(floor((d_i - self.d_min) / self.delta_d))
            j = int(floor((phi_i - self.phi_min) / self.delta_phi))
            measurement_likelihood[i, j] += weight
        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        measurement_likelihood = measurement_likelihood / np.sum(measurement_likelihood)
        return measurement_likelihood

    def getEstimateList(self):
        d_max = np.zeros(self.num_belief)
        phi_max = np.zeros(self.num_belief)
        for i in range(self.num_belief):
            maxids = np.unravel_index(self.beliefArray[i].argmax(), self.beliefArray[i].shape)
            # add 0.5 because we want the center of the cell
            d_max[i] = self.d_min + (maxids[0] + 0.5) * self.delta_d
            phi_max[i] = self.phi_min + (maxids[1] + 0.5) * self.delta_phi
        return d_max, phi_max

    def get_estimate(self):
        d_max, phi_max = self.getEstimateList()
        d_median = np.median(d_max[1:])
        phi_median = np.median(phi_max[1:])
        res = OrderedDict()
        res['d'] = d_median
        res['phi'] = phi_median
        return res

    def getEstimate(self):
        """ Returns a list with two elements: (d, phi) """
        res = self.get_estimate()
        return [res['d'], res['phi']]

    def getMax(self):
        return self.beliefArray[0].max()

    def get_entropy(self):
        s = entropy(self.beliefArray[0].flatten())
        return s

    def generateVote(self, segment):
        """
            return d_i, phi_i, l_i, weight

            XXX: What is l_i?
        """
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        distance = np.linalg.norm(p2 - p1)
        t_hat = (p2 - p1) / distance
        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1 + l2) / 2
        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE:  # right lane is white
            if(p1[0] > p2[0]):  # right edge of white lane
                d_i = d_i - self.linewidth_white
            else:  # left edge of white lane
                d_i = -d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth / 2

        elif segment.color == segment.YELLOW:  # left lane is yellow
            if (p2[0] > p1[0]):  # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else:  # right edge of white lane
                d_i = -d_i
            d_i = self.lanewidth / 2 - d_i

        # weight = distance
        weight = 1
        return d_i, phi_i, l_i, weight

    def get_plot_phi_d(self, ground_truth=None):  # @UnusedVariable
        est = self.get_estimate()
        belief = self.beliefArray[0]
        other_d, other_phi = self.getEstimateList()
        return plot_phi_d_diagram_bgr(self, belief, phi=est['phi'], d=est['d'],
                                      other_phi=other_phi, other_d=other_d)

    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return np.hypot(x_c, y_c)
