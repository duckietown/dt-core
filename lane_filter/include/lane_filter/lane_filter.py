from duckietown_utils.parameters import Configurable
from duckietown_msgs.msg import Segment
import numpy as np
from .lane_filter_interface import LaneFilterInterface
from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, pi, sqrt
import copy



class LaneFilterHistogram(Configurable, LaneFilterInterface):
    """LaneFilterHistogram"""

    def __init__(self,configuration):
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
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self,param_names,configuration)

        self.num_belief = 7
        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefArray = []
        for i in range(self.num_belief):
            self.beliefArray.append(np.empty(self.d.shape))
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0  = [ [self.sigma_d_0, 0], [0, self.sigma_phi_0] ]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]  
         
        self.initialize()
        
        
    def predict(self, dt, v, w):
        delta_t = dt
        d_t = self.d + v*delta_t*np.sin(self.phi)
        phi_t = self.phi + w*delta_t

        for k in range(self.num_belief):
            p_belief = np.zeros(self.beliefArray[k].shape)

            # there has got to be a better/cleaner way to do this - just applying the process model to translate each cell value
            for i in range(self.beliefArray[k].shape[0]):
                for j in range(self.beliefArray[k].shape[1]):
                    if self.beliefArray[k][i,j] > 0:
                        if d_t[i,j] > self.d_max or d_t[i,j] < self.d_min or phi_t[i,j] < self.phi_min or phi_t[i,j] > self.phi_max:
                            continue
                        i_new = int(floor((d_t[i,j] - self.d_min)/self.delta_d))
                        j_new = int(floor((phi_t[i,j] - self.phi_min)/self.delta_phi))
                        p_belief[i_new,j_new] += self.beliefArray[k][i,j]

            s_belief = np.zeros(self.beliefArray[k].shape)
            gaussian_filter(p_belief, self.cov_mask, output=s_belief, mode='constant')

            if np.sum(s_belief) == 0:
                return
            self.beliefArray[k] = s_belief/np.sum(s_belief)


    
    def update(self, segments, range_arr):

        for i in range(self.num_belief):
            if i == 0:
                measurement_likelihood = self.generate_measurement_likelihood(segments, range_arr[i], range_arr[i + 3])    
            else:
                measurement_likelihood = self.generate_measurement_likelihood(segments, range_arr[i], range_arr[i + 1])
            if measurement_likelihood is not None:
                self.beliefArray[i] = np.multiply(self.beliefArray[i],measurement_likelihood)
                if np.sum(self.beliefArray[i]) == 0:
                    self.beliefArray[i] = measurement_likelihood
                else:
                    self.beliefArray[i] = self.beliefArray[i]/np.sum(self.beliefArray[i])

    def generate_measurement_likelihood(self, segments, range_min, range_max):
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape)
        for segment in segments:
            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            point_range = self.getSegmentDistance(segment)
            # print "Point range: ", point_range 

            # only consider points in a certain range from the Duckiebot
            if point_range < range_min or point_range > range_max:
                continue
            d_i,phi_i,l_i = self.generateVote(segment)
            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            i = int(floor((d_i - self.d_min)/self.delta_d))
            j = int(floor((phi_i - self.phi_min)/self.delta_phi))
            measurement_likelihood[i,j] = measurement_likelihood[i,j] +  1 
        if np.linalg.norm(measurement_likelihood) == 0:            
            return None
        measurement_likelihood = measurement_likelihood/np.sum(measurement_likelihood)
        return measurement_likelihood
        
    def getEstimate(self):
        d_max = np.zeros(self.num_belief)
        phi_max = np.zeros(self.num_belief)
        for i in range(self.num_belief):
            maxids = np.unravel_index(self.beliefArray[i].argmax(),self.beliefArray[i].shape)
            d_max[i] = self.d_min + maxids[0]*self.delta_d
            phi_max[i] = self.phi_min + maxids[1]*self.delta_phi
        return [d_max,phi_max]

    def getMax(self):
        return self.beliefArray[0].max()

    def initialize(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        for i in range(self.num_belief):
            self.beliefArray[i]=RV.pdf(pos)


    def generateVote(self,segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2-p1)/np.linalg.norm(p2-p1)
        n_hat = np.array([-t_hat[1],t_hat[0]])
        d1 = np.inner(n_hat,p1)
        d2 = np.inner(n_hat,p2)
        l1 = np.inner(t_hat,p1)
        l2 = np.inner(t_hat,p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1+l2)/2
        d_i = (d1+d2)/2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE: # right lane is white
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth/2

        elif segment.color == segment.YELLOW: # left lane is yellow
            if (p2[0] > p1[0]): # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else: # right edge of white lane
                d_i = -d_i
            d_i =  self.lanewidth/2 - d_i

        return d_i, phi_i, l_i

    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x)/2
        y_c = (segment.points[0].y + segment.points[1].y)/2
        return sqrt(x_c**2 + y_c**2)

