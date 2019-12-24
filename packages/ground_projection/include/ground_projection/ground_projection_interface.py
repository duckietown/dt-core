from collections import namedtuple
import os.path

import cv2

from duckietown_msgs.msg import Segment, SegmentList
import duckietown_utils as dtu
from ground_projection.configuration import get_extrinsics_filename
import numpy as np
from pi_camera import get_camera_info_for_robot

from .configuration import get_homography_for_robot
from .ground_projection_geometry import GroundProjectionGeometry

__all__ = [
    'GroundProjection',
]


@dtu.memoize_simple
def get_ground_projection(robot_name):
    return GroundProjection(robot_name)


@dtu.contract(returns=GroundProjectionGeometry, robot_name=str)
def get_ground_projection_geometry_for_robot(robot_name):
    gp = get_ground_projection(robot_name)
    return gp.get_ground_projection_geometry()


class GroundProjection(object):

    def __init__(self, robot_name):
        camera_info = get_camera_info_for_robot(robot_name)
        homography = get_homography_for_robot(robot_name)
        self._gpg = GroundProjectionGeometry(camera_info, homography)

        self.robot_name = robot_name

    @dtu.contract(returns=GroundProjectionGeometry)
    def get_ground_projection_geometry(self):
        return self._gpg

    def get_camera_info(self):
        return self._gpg.ci


class CouldNotCalibrate(Exception):
    pass


HomographyEstimationResult = namedtuple('HomographyEstimationResult',
                                        'success error board_info bgr_detected bgr_detected_refined H')

def get_corners(grey_rectified):
    def select(img,x,y, radius):
        prev_state = None
        counts = 0
        for angle in range(0, 360,1):
            x_perimeter = int(x + np.sin(angle/180.*np.pi) * 3*radius)
            y_perimeter = int(y + np.cos(angle/180.*np.pi) * radius)

            state = img[y_perimeter,x_perimeter]<20
            if prev_state != state:
                counts+=1
                prev_state = state
        return counts>3
    corners = cv2.goodFeaturesToTrack(grey_rectified,50,.1,7)
    corners = np.int0(corners)
    
    kept_corners = []
    for i in corners:
        x,y = i.ravel()
        if select(grey_rectified,x,y, radius=5):
            kept_corners.append([x,y])
    return kept_corners

def clean_points(points, threshold = 8.0):
    # TODO: Add credits
    def squared_distance(p1, p2):
        # TODO optimization: use numpy.ndarrays, simply return (p1-p2)**2
        sd = 0
        for x, y in zip(p1, p2):
            sd += (x-y)**2
        return sd


    def get_proximity_matrix(points, threshold):
        n = len(points)
        t2 = threshold**2
        # TODO optimization: use sparse boolean matrix
        prox = [[False]*n for k in range(n)]
        for i in range(0, n):
            for j in range(i+1, n):
                prox[i][j] = (squared_distance(points[i], points[j]) < t2)
                prox[j][i] = prox[i][j]  # symmetric matrix
        return prox


    def find_clusters(points, threshold):
        n = len(points)
        prox = get_proximity_matrix(points, threshold)
        point_in_list = [None]*n
        clusters = []
        for i in range(0, n):
            for j in range(i+1, n):
                if prox[i][j]:
                    list1 = point_in_list[i]
                    list2 = point_in_list[j]
                    if list1 is not None:
                        if list2 is None:
                            list1.append(j)
                            point_in_list[j] = list1
                        elif list2 is not list1:
                            # merge the two lists if not identical
                            list1 += list2
                            point_in_list[j] = list1
                            del clusters[clusters.index(list2)]
                        else:
                            pass  # both points are already in the same cluster
                    elif list2 is not None:
                        list2.append(i)
                        point_in_list[i] = list2
                    else:
                        list_new = [i, j]
                        for index in [i, j]:
                            point_in_list[index] = list_new
                        clusters.append(list_new)
            if point_in_list[i] is None:
                list_new = [i]  # point is isolated so far
                point_in_list[i] = list_new
                clusters.append(list_new)
        return clusters


    def average_clusters(points, threshold=1.0, clusters=None):
        if clusters is None:
            clusters = find_clusters(points, threshold)
        newpoints = []
        for cluster in clusters:
            n = len(cluster)
            point = [0]*len(points[0])  # TODO numpy
            for index in cluster:
                part = points[index]  # in numpy: just point += part / n
                for j in range(0, len(part)):
                    point[j] += part[j] / n  # TODO optimization: point/n later
            newpoints.append(point)
        return newpoints

    clusters = find_clusters(points, threshold)
    clustered = average_clusters(points, clusters=clusters)
    return np.array(clustered).astype(np.float32).reshape(-1,1,2)

def order_corners(corners, n_points, step=1.):
    ordered_corners = []
    remaining_points = corners.astype(np.float32).reshape(-1,2).tolist()

    while len(remaining_points)>0:

        left_idx = np.argmin(list(map(lambda x:x[0]-x[1],remaining_points)))      
        left_point = remaining_points[left_idx]

        right_idx = np.argmax(list(map(lambda x:x[0]+x[1],remaining_points)))      
        right_point = remaining_points[right_idx]
    
        slope = (left_point[1] - right_point[1])/(left_point[0] - right_point[0])
        offset = left_point[1] - left_point[0]*slope
        direction_pos = True
        while True:
            top_points = []
            n_points_above = 0
            for idx, point in enumerate(remaining_points):
                if point[1] - point[0]*slope - offset>0:
                    n_points_above+=1
                    top_points.append(point)
            if len(top_points) < n_points:
                if not direction_pos:
                    step*=.5
                    direction_pos = True
                offset-=step
            elif len(top_points) == n_points:
                ordered_corners+=sorted(top_points, key=lambda x:-x[0])
                for p in top_points:
                    remaining_points.remove(p)
                break
            else:
                if direction_pos:
                    step*=.5
                    direction_pos = False
                offset+=step

    return np.array(ordered_corners[::-1]).astype(np.float32).reshape(-1,1,2)

def estimate_homography(bgr_rectified):
    '''
        Estimate ground projection using instrinsic camera calibration parameters.

        Returns HomographyEstimationResult
    '''

    assert bgr_rectified.shape == (480, 640, 3)
    grey_rectified = cv2.cvtColor(bgr_rectified, cv2.COLOR_BGR2GRAY)

    board = load_board_info()
    board_width = board['width']
    board_height = board['height']
    square_size = board['square_size']
    board_offset = board['offset']
    dtu.logger.info('board: \n %s' % board)

    # Defaults
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH
    #+ cv2.CALIB_CB_NORMALIZE_IMAGE
    #flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    #flags = cv2.CALIB_CB_NORMALIZE_IMAGE

    pattern = (board_width, board_height)
    the_input = grey_rectified.copy()
    # ----- cv2.findChessboardCorner couldn't detect corners on the simulator's images
    #ret, corners = cv2.findChessboardCorners(the_input, pattern, flags=flags)
    # ----- work around: develop our own corners detector
    the_input = cv2.GaussianBlur(the_input,(5,5),1)
    corners = get_corners(the_input)
    corners = clean_points(corners,threshold = 8.)
    ret = pattern[0]*pattern[1] == len(corners)

#    bgr_detected = cv2.cvtColor(grey_rectified, cv2.COLOR_GRAY2BGR)
    bgr_detected = bgr_rectified.copy()
    cv2.drawChessboardCorners(bgr_detected, (7, 5), corners, ret)
    
    if ret == False:
        msg = "findChessboardCorners failed (len(corners) == %s)" % (len(corners) if corners is not None else 'none')
        return HomographyEstimationResult(success=False,
                                          error=msg,
                                          board_info=board,
                                          bgr_detected=bgr_detected,
                                          bgr_detected_refined=None,
                                          H=None)

    expected = board_width * board_height
    if len(corners) != expected:
        msg = "Not all corners found in image. Expected: %s; found: %s" % (expected, len(corners))
        dtu.raise_desc(CouldNotCalibrate, msg)

    #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    #corners2 = cv2.cornerSubPix(grey_rectified, corners, (11, 11), (-1, -1), criteria)

    corners = order_corners(corners, n_points=pattern[0])

#    bgr_detected_refined = cv2.cvtColor(grey_rectified, cv2.COLOR_GRAY2BGR)
    bgr_detected_refined = bgr_rectified.copy()
    cv2.drawChessboardCorners(bgr_detected_refined, (7, 5), corners, ret)

    src_pts = []
    for r in range(board_height):
        for c in range(board_width):
            src_pts.append(np.array([r * square_size , c * square_size] , dtype='float32')
                           + board_offset)

    # OpenCV labels corners left-to-right, top-to-bottom
    # We're having a problem with our pattern since it's not rotation-invariant

    # only reverse order if first point is at bottom right corner
    if ((corners[0])[0][0] < (corners[board_width * board_height - 1])[0][0] and \
        (corners[0])[0][0] < (corners[board_width * board_height - 1])[0][1]):
        dtu.logger.info("Reversing order of points.")
        src_pts.reverse()

    # Compute homography from image to ground
    H, _mask = cv2.findHomography(corners.reshape(len(corners), 2), np.array(src_pts), cv2.RANSAC)

    return HomographyEstimationResult(success=True,
                                      error=None,
                                      board_info=board,
                                      bgr_detected=bgr_detected,
                                      bgr_detected_refined=bgr_detected_refined,
                                      H=H)


def save_homography(H, robot_name):
    dtu.logger.info('Homography:\n %s' % H)

    # Check if specific point in matrix is larger than zero (this would definitly mean we're having a corrupted rotation matrix)
    if(H[1][2] > 0):
        msg = "WARNING: Homography could be corrupt."
        msg += '\n %s' % H
        raise Exception(msg)

    ob = {'homography': sum(H.reshape(9, 1).tolist(), [])}

    import yaml as alt
    import time
    s = alt.dump(ob)
    s += "\n# Calibrated on %s"%dtu.format_time_as_YYYY_MM_DD(time.time())

    fn = get_extrinsics_filename(robot_name)

    dtu.write_data_to_file(s, fn)

#    dtu.yaml_write_to_file(ob, extrinsics_filename)


@dtu.contract(sl=SegmentList, gpg=GroundProjectionGeometry, returns=SegmentList)
def find_ground_coordinates(gpg, sl, skip_not_on_ground=True):
    """
        Creates a new segment list with the ground coordinates set.

    """
    cutoff = 0.01
    sl2 = SegmentList()
    sl2.header = sl.header

    # Get ground truth of segmentList
    for s1 in sl.segments:
        g0 = gpg.vector2ground(s1.pixels_normalized[0])
        g1 = gpg.vector2ground(s1.pixels_normalized[1])
        if skip_not_on_ground:
            if g0.x < cutoff or g1.x < cutoff:
                continue

        points = [g0, g1]
        pixels_normalized = [s1.pixels_normalized[0], s1.pixels_normalized[1]]
        color = s1.color
        s2 = Segment(points=points, pixels_normalized=pixels_normalized, color=color)
        # TODO what about normal and points
        sl2.segments.append(s2)
    return sl2


def load_board_info(filename=None):
    '''Load calibration checkerboard info'''
    if filename is None:
        root = dtu.get_ros_package_path('ground_projection')
        filename = root + '/config/ground_projection_node/default.yaml'

    if not os.path.isfile(filename):
        msg = 'No such file: %s' % filename
        raise dtu.DTException(msg)

    target_data = dtu.yaml_load_file(filename)
    target_info = {
        'width': target_data['board_w'],
        'height': target_data['board_h'],
        'square_size': target_data['square_size'],
        'x_offset': target_data['x_offset'],
        'y_offset': target_data['y_offset'],
        'offset': np.array([target_data['x_offset'], -target_data['y_offset']]),
        'size': (target_data['board_w'], target_data['board_h']),
      }
    dtu.logger.info("Loaded checkerboard parameters")
    return target_info

