from comptests.registrar import comptest, run_module_tests

import cv2

from complete_image_pipeline.pipeline import run_pipeline
from duckietown_segmaps.draw_map_on_images import plot_map
from duckietown_segmaps.maps import FRAME_AXLE, FRAME_GLOBAL
from duckietown_segmaps.transformations import TransformationsInfo
import duckietown_utils as dtu
from easy_algo.algo_db import get_easy_algo_db
from ground_projection import GroundProjection
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
from localization_templates import FAMILY_LOC_TEMPLATES
from localization_templates import TemplateStraight
import numpy as np


template = 'DT17_straight_straight'
robot_name = 'flitzer'
line_detector_name = 'baseline'
# image_prep_name = 'prep_200_100'
image_prep_name = 'baseline'
# lane_filter_names = ['baseline', 'generic_straight']
lane_filter_names = []
lane_filter_names += ['generic_straightstraight']
lane_filter_names += ['baseline']
raise_if_error_too_large = True

max_phi_err = np.deg2rad(5)
max_d_err = 0.021



dirn = lambda _: 'out-synthetic/%s' % _
@comptest
def test_synthetic_zero_zerophi():
    d = 0
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_zerophi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                   image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_pos_zerophi():
    d = 0.05
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_pos_zerophi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_neg_posphi():
    d = -0.05
    phi = np.deg2rad(15)
    
    for lane_filter_name in lane_filter_names:
        outd = dirn('test_synthetic_neg_posphi-' + lane_filter_name) 
        test_synthetic_phi(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_zero_posphi():
    d = 0
    phi = np.deg2rad(15)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_posphi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

@comptest
def test_synthetic_zero_negphi():
    d = 0
    phi = np.deg2rad(-20)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_negphi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

@comptest
def test_synthetic_zero_bignegphi():
    d = 0
    phi = np.deg2rad(-50)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_bignegphi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                           image_prep_name, lane_filter_name, d, phi, outd)

@comptest
def test_synthetic_zero_bigposphi():
    d = 0
    phi = np.deg2rad(+50)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_bignegphi-' + lane_filter_name)
        test_synthetic_phi(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi, outd)

def test_synthetic_phi(template,robot_name,line_detector_name,
                   image_prep_name, lane_filter_name, d, phi, outd,
                   max_phi_err=max_phi_err, 
                   max_d_err=max_d_err):

    location = np.zeros((), dtype=TemplateStraight.DATATYPE_COORDS)
    location['phi'] = phi 
    location['d'] = d 
    
    _res, stats = test_synthetic(template,robot_name,line_detector_name,
                                image_prep_name, lane_filter_name, location, outd)
    error = stats['error']
    estimate = stats['estimate']
    fail = False
    msg = 'location: %s  estimate: %s error: %s ' % (location, estimate, error)
    if np.abs(error['phi']) > max_phi_err:
        msg += '\nError in phi too big (%s > %s) ' % (np.abs(error['phi']), max_phi_err)
        fail = True
    if np.abs(error['d']) > max_d_err:
        msg += '\nError in d too big (%s > %s)' % (np.abs(error['d']), max_d_err)
        fail = True
    if fail:
        dtu.logger.error(msg)
    
        if raise_if_error_too_large:
            raise Exception(msg)
    
     
def test_synthetic(template, robot_name,line_detector_name,
                   image_prep_name, lane_filter_name, location, outd):

    # first, load calibration for robot
    easy_algo_db = get_easy_algo_db()
    dtu.logger.debug('looking for localization template %r' % template)
    localization_template = easy_algo_db.create_instance(FAMILY_LOC_TEMPLATES, template)
    
    gp = GroundProjection(robot_name)
    # GroundProjectionGeometry
    gpg = gp.get_ground_projection_geometry() 
    
    pose = localization_template.pose_from_coords(location)
    sm_orig = localization_template.get_map()
    distorted_synthetic = simulate_image(sm_orig, pose, gpg, blur_sigma=0.3)

    image = distorted_synthetic

    res, stats = run_pipeline(image, gp, line_detector_name, image_prep_name, lane_filter_name,
                       all_details=False,
                       skip_instagram=True, ground_truth=pose)
    
    error = np.empty_like(location)
    for k in error.dtype.fields:
        error[k] = stats['estimate'][k] - location[k]
    stats['error'] = error

    res = dtu.resize_small_images(res)
    
    dtu.write_bgr_images_as_jpgs(res, outd, extra_string=outd.split('/')[-1])
    return res, stats

@dtu.contract(gpg=GroundProjectionGeometry, pose='SE2')
def simulate_image(sm_orig, pose, gpg, blur_sigma):
    camera_info = gpg.get_camera_info() 
    blank = generate_blank(camera_info)
    tinfo = TransformationsInfo()

    frames = list(set(_.id_frame for _ in sm_orig.points.values()))    
    id_frame = frames[0]
    dtu.logger.debug('frames: %s choose %s' % (frames, id_frame))

    tinfo.add_transformation(frame1=id_frame, frame2=FRAME_AXLE, g=pose) 
        
    sm_axle = tinfo.transform_map_to_frame(sm_orig, FRAME_AXLE)
    
    rectified_synthetic = plot_map(blank, sm_axle, gpg, do_segments=False)
    
    distorted_synthetic = gpg.distort(rectified_synthetic)
    
    distorted_synthetic = add_noise(distorted_synthetic)
#     distorted_synthetic = cv2.medianBlur(distorted_synthetic, 3)
    distorted_synthetic = cv2.GaussianBlur(distorted_synthetic, (0,0), blur_sigma)
    return distorted_synthetic
    
def add_noise(image, intensity = 20, noise_blur = 1):
    noise = np.random.randn(image.shape[0], image.shape[1], 3) * intensity
    noise = cv2.GaussianBlur(noise, (0,0), noise_blur)
    image = image*1.0 + noise
    image = image.clip(0, 255).astype('uint8')
    return image
    
def generate_blank(camera_info):
    H = camera_info.height
    W = camera_info.width
    bgr = np.zeros(dtype='uint8', shape=(H, W, 3))
    bgr.fill(128)
    return bgr
    
if __name__ == '__main__':
    run_module_tests()