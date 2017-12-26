from collections import OrderedDict
import collections
import itertools

import duckietown_utils as dtu
import numpy as np


__all__ = [
    'GridHelper',
]

VotingGridVarSpec = collections.namedtuple('VotingGridVarSpec',
                        'min max resolution description units units_display')

class GridHelper(object):
    """
        This class abstracts the task of creating a grid that
        is used for voting.
    """
    @dtu.contract(variables='dict(str:dict)')
    def __init__(self, variables):
        dtu.check_isinstance(variables, OrderedDict)
        if len(variables) != 2:
            msg = 'I can only deal with 2 variables, obtained %s' % self._variables
            raise ValueError(msg)

        self._names = list(variables)
        self._specs = [spec_from_yaml(variables[_]) for _ in self._names]
            
        
        s0 = self._specs[0]
        s1 = self._specs[1]
        self._mgrids = list(np.mgrid[s0.min:s0.max:s0.resolution,
                                     s1.min:s1.max:s1.resolution])
        H, W = self.shape = self._mgrids[0].shape 
        self._centers = [np.zeros(shape=(H, W)),np.zeros(shape=(H, W))]
        
        for i, j in itertools.product(range(H), range(W)):
            self._centers[0][i,j] = s0.min + (i+0.5)*s0.resolution
            self._centers[1][i,j] = s1.min + (j+0.5)*s1.resolution

        # these are the bounds you would give to pcolor
        # there is one row and one column more
        # self.d, self.phi are the lower corners

        # Each cell captures this area:
        #         (X[i,   j],   Y[i,   j]),
        #         (X[i,   j+1], Y[i,   j+1]),
        #         (X[i+1, j],   Y[i+1, j]),
        #         (X[i+1, j+1], Y[i+1, j+1])
        self._mgrids_plus = list(np.mgrid[s0.min:s0.max+s0.resolution:s0.resolution,
                                          s1.min:s1.max+s1.resolution:s1.resolution])

        self.K0 = lambda x: gaussian_kernel(x, self._specs[0].resolution)
        self.K1 = lambda x: gaussian_kernel(x, self._specs[1].resolution)
        
    def get_shape(self):
        """ Returns the shape of the grid """
        return self.shape
    
    def create_new(self, dtype='float64'):
        """ Creates a new numpy array of compatible dimensions, set to NaN """
        res = np.zeros(shape=self.shape, dtype=dtype)
        res.fill(np.nan)
        return res
    
    @dtu.contract(target='array', value=dict)
    def add_vote(self, target, value, weight, F = 1):
        """ Returns 
            
                value = dict(phi=2,d=3)
                weight = 1
                hit = x.add_vote(grid, value, weight)
        """
        values = [value[_] for _ in self._names]
        
        # Check if inside bounds
        coords = [None, None] # i, j
        for a in range(2):
            spec = self._specs[a]
            inside = spec.min <= values[a] <= spec.max
            if not inside:
#                 dtu.logger.debug('Not inside bound %s <= %s < %s' % 
#                              (spec.min, values[a], spec.max))
                return 0
            
            x = (values[a] - spec.min) / spec.resolution
            coords[a] = int(np.floor(x))
            
        i, j = coords
        
        H, W = self.shape
        targets = []
        weights = []
        for di, dj in itertools.product(range(-F,F+1), range(-F,F+1)):
            u = i + di
            v = j + dj
            
            if not ((0<=u<H) and (0<=v<W)):
                continue
            
            v0_cell = self._centers[0][u,v]
            v1_cell = self._centers[1][u,v]
            v0_delta = values[0] - v0_cell
            v1_delta = values[1] - v1_cell
            
            k = self.K0(v0_delta) * self.K1(v1_delta)
            weight_k = weight * k
            
            targets.append((u, v))
            weights.append(weight_k)
        
        if weights:    
            total_weight = sum(weights)
         
        for (u,v), weight in zip(targets, weights):
            weight_k = weight / total_weight
            target[u, v] += weight_k       

        return 1
    
    
    def get_max(self, target):
        """ Returns a dictionary """
        assert self.shape == target.shape
        amax = target.argmax()
        check_no_nans(target)
        maxids = np.unravel_index(amax, target.shape)
        i, j = maxids
        d = OrderedDict()
        d[self._names[0]] = self._centers[0][i,j]
        d[self._names[1]] = self._centers[1][i,j]
        return d
    
    def get_max_weighted(self, target, F=1):
        assert self.shape == target.shape
        check_no_nans(target)
        amax = target.argmax()
        maxids = np.unravel_index(amax, target.shape)
        i, j = maxids
        H, W = self.shape
        v0 = []
        v1 = []
        weights = []

        for di, dj in itertools.product(range(-F,F+1), range(-F,F+1)):
            u = i + di
            v = j + dj
            
            if not ((0<=u<H) and (0<=v<W)):
                continue
            
            v0.append(self._centers[0][u,v])
            v1.append(self._centers[1][u,v])
            weights.append(target[u, v])
        
        v0 = np.array(v0)
        v1 = np.array(v1)
        s = np.sum(weights)
        mean0 = np.sum(v0*weights) / s 
        mean1 = np.sum(v1*weights) / s
            
        d = OrderedDict()
        d[self._names[0]] = mean0
        d[self._names[1]] = mean1
        return d

def gaussian_kernel(x, sigma):
    d = x/sigma
    return np.exp(-np.power(d, 2))


def spec_from_yaml(spec):
    vmin = spec['min']
    vmax = spec['max']
    resolution = spec['resolution']
    description = spec['description']
    units = spec['units']
    units_display = spec.get('units_display', units)

    return VotingGridVarSpec(min=vmin, max=vmax, resolution=resolution,
                             description=description, units=units,
                             units_display=units_display)
    
def check_no_nans(target):
    if np.any(np.isnan(target)):
        msg = 'I found some NaNs'
        msg += '\n' + str(target)
        raise ValueError(msg)
    
        