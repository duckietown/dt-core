from collections import namedtuple

from numpy.testing.utils import assert_almost_equal

import duckietown_utils as dtu
from duckietown_utils.matplotlib_utils import CreateImageFromPylab
import numpy as np
from geometry.poses import translation_angle_from_SE2


SegMapPoint = namedtuple('SegMapPoint', 'id_frame coords') 
SegMapSegment = namedtuple('SegMapSegment', 'points color')
SegMapFace = namedtuple('SegMapFace', 'points color')

FAMILY_SEGMAPS = 'segmap'
FRAME_AXLE = 'axle'
FRAME_TILE = 'tile'
FRAME_GLOBAL = 'global'

__all__ = [
    'SegMapSegment',
    'SegMapPoint',
    'SegMapFace',
    'FRAME_AXLE',
    'FRAME_TILE',
    'FRAME_GLOBAL',
    'FAMILY_SEGMAPS',
]

class SegmentsMap(object):
    
    @dtu.contract(points=dict, segments=list, faces=list, constants='None|dict(str:float)')
    def __init__(self, points, segments, faces, constants=None):
        if constants is None:
            constants = {}
        self.points = points
        self.segments = segments
        self.faces = faces
        self.constants = constants
        self.validate()
        
    @staticmethod
    def empty():
        return SegmentsMap(points={}, segments=[], faces=[], constants={})
        
    def validate(self):
        for k, p in self.points.items():
            dtu.check_isinstance(p, SegMapPoint)
            dtu.check_isinstance(k, str)
            
            coords = p.coords
            if not isinstance(coords, np.ndarray):
                self.points[k] = p._replace(coords=np.array(coords))
            
        for S in self.segments:
            dtu.check_isinstance(S, SegMapSegment)
            for p in S.points:
                if not p in self.points:
                    msg = 'Invalid point %r' % p
                    raise ValueError(msg)
            assert(len(S.points) == 2)
                
            w1 = self.points[S.points[0]].coords
            w2 = self.points[S.points[1]].coords
            
            dist = np.linalg.norm(w1-w2)
            if dist == 0:
                msg = 'This is a degenerate segment (points: %s %s) ' % (w1, w2)
                msg += 'names: %s' % S.points
                raise ValueError(msg)


        for F in self.faces:
            dtu.check_isinstance(F, SegMapFace)
            for p in F.points:
                if not p in self.points:
                    msg = 'Invalid point %r' % p
                    raise ValueError(msg)

    @staticmethod
    def merge(sms):
        points = {}
        segments = []
        faces = []
        constants = {}
        for i, sm in enumerate(sms):
            # first, make sure the names are different
            sm2 = add_prefix(sm, 'm%d_'%i)
            points.update(sm2.points)
            segments.extend(sm2.segments)
            faces.extend(sm2.faces)
            # XXX: what if conflict?
            constants.update(sm2.constants)

        return SegmentsMap(points=points, faces=faces, 
                           segments=segments, constants=constants)
        
    @staticmethod
    def from_yaml(data):
        points = data['points']
        faces = data['faces']
        segments = data['segments']
        
        points2 = {}
        for k, p in points.items():
            points2[k] = SegMapPoint(id_frame=p[0], coords=np.array(p[1]))
        
        segments2 = []
        for S in segments:
            if not isinstance(S, SegMapSegment):
                S = SegMapSegment(points=S['points'], color=S['color'])
            segments2.append(S)
        
        faces2 = []
        for f in faces:
            F = SegMapFace(points=f['points'], color=f['color'])
            faces2.append(F)

        return SegmentsMap(points=points2, faces=faces2, segments=segments2)


@dtu.contract(sm=SegmentsMap, ground_truth='SE2|None')
def plot_map_and_segments(sm, tinfo, segments, dpi=120, ground_truth=None):
    """ Returns a BGR image """  
    a = CreateImageFromPylab(dpi=dpi)

    with a as pylab:
        _plot_map_segments(sm, pylab, FRAME_GLOBAL)
        _plot_detected_segments(tinfo, segments, pylab)
        
        # draw arrow
        L = 0.1

        if ground_truth is not None:
            (x, y), _ = translation_angle_from_SE2(ground_truth)
            x1, y1, _ = np.dot(ground_truth, [L, 0, 1])
#             x = ground_truth['x']
#             y = ground_truth['y']
#             theta = ground_truth['theta']
#             x1 = x + L * np.cos(theta)
#             y1 = y + L * np.sin(theta)
            pylab.plot(x, y, 'co', markersize=12)
            pylab.plot([x, x1], [y, y1], 'c-', linewidth=4)
#             pylab.plot([-1, +1], [y, y], 'c--', markersize=10)
#             print('ground truth: %s %s %s' % (x,y,np.rad2deg(theta)))
            
        w1 = tinfo.transform_point(np.array([0,0,0]), frame1=FRAME_AXLE, frame2=FRAME_GLOBAL)
        w2 = tinfo.transform_point(np.array([L,0,0]), frame1=FRAME_AXLE, frame2=FRAME_GLOBAL)
        
        pylab.plot([w1[0], w2[0]], [w1[1], w2[1]], 'm-')
        pylab.plot(w1[0], w1[1], 'mo', markersize=6)
        
        pylab.axis('equal')


    return a.get_bgr()



def _plot_detected_segments(tinfo, segments, pylab):
    
    for segment in segments:
        p1 = segment.points[0]
        p2 = segment.points[1]
        
        f = lambda _: np.array([_.x, _.y, _.z])
        
        w1 = tinfo.transform_point(f(p1), frame1=FRAME_AXLE, frame2=FRAME_GLOBAL)
        w2 = tinfo.transform_point(f(p2), frame1=FRAME_AXLE, frame2=FRAME_GLOBAL)
        
#         dtu.logger.debug('Plotting w1 %s w2 %s' % (w1, w2))
        pylab.plot([w1[0], w2[0]], [w1[1], w2[1]], 'm-')

        
@dtu.contract(sm=SegmentsMap)
def _plot_map_segments(sm, pylab, expect_frame, plot_ref_segments=True):

    for face in sm.faces:
        xs = []
        ys = []
        for p in face.points:
            if sm.points[p].id_frame != expect_frame:
                msg = "Expected points in frame %r, got %r" %(expect_frame, sm.points[p].id_frame)
                raise NotImplementedError(msg)
            
            coords = sm.points[p].coords
            xs.append(coords[0])
            ys.append(coords[1])
            
        xs.append(xs[0])
        ys.append(ys[0])
        facecolor = face.color
        edgecolor = 'none'
        pylab.fill(xs, ys, facecolor=facecolor, edgecolor=edgecolor)

    if plot_ref_segments:
        for segment in sm.segments:
            p1 = segment.points[0]
            p2 = segment.points[1]
    
            # If we are both in FRAME_AXLE
            if ( not (sm.points[p1].id_frame == expect_frame) and 
                 (sm.points[p2].id_frame == expect_frame)):
                msg = "Cannot deal with points not in frame %r" % expect_frame
                raise NotImplementedError(msg)
        
            w1 = np.array(sm.points[p1].coords)
            w2 = np.array(sm.points[p2].coords)
            
            color = 'c-'
            pylab.plot([w1[0], w2[0]], [w1[1], w2[1]], color)
        
            n  = get_normal_outward_for_segment(w1, w2)
            center = 0.51*w1 + 0.49*w2
            L = 0.05
            
            center2 = center + n * L
            
            pylab.plot([center[0], center2[0]], [center[1], center2[1]], color)
            
        
def get_normal_outward_for_segment(w1, w2):
    """ Outward points towards black """
    d = w2 - w1
    #   w2
    #   |  --->
    #   w1
    dn = np.linalg.norm(d)
    if dn == 0:
        msg = 'Could not compute normal for segment with two points equal:'
        msg += ' %s %s' % (w1,w2)
        raise ValueError(msg)
    d = d / dn
    
    z = np.array([0,0,1])
    c = np.cross(d, z)
    assert_almost_equal(np.linalg.norm(c), 1.0)
    return c

# TODO: move away
assert_almost_equal(np.array([0,-1,0]), 
                    get_normal_outward_for_segment(np.array([0,0,0]), np.array([2,0,0])))


def add_prefix(sm, prefix):
    points = {}
    faces = []
    segments = []
    for k, v in sm.points.items():
        points[prefix + k] = v
    for face in sm.faces:
        points2 = tuple(prefix +_ for _ in face.points)
        faces.append(face._replace(points=points2))
    for segment in sm.segments:
        points2 = tuple(prefix +_ for _ in segment.points)
        segments.append(segment._replace(points=points2))
        
    return SegmentsMap(points=points, faces=faces, segments=segments, 
                       constants=sm.constants)
