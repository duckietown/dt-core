from comptests.registrar import comptest, run_module_tests

from duckietown_segmaps.draw_map_on_images import plot_map
from duckietown_segmaps.map_localization_template import FAMILY_LOC_TEMPLATES
from duckietown_segmaps.maps import FRAME_AXLE, FRAME_TILE
from duckietown_segmaps.template_lane_straight import TemplateStraightLane
from duckietown_segmaps.transformations import TransformationsInfo
import duckietown_utils as dtu
from easy_algo.algo_db import get_easy_algo_db
from ground_projection import GroundProjection
import numpy as np
import cv2
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
from complete_image_pipeline.pipeline import run_pipeline

template = 'DT17_ETH_straight'
robot_name = 'flitzer'
line_detector_name = 'baseline'
image_prep_name = 'prep_200_100'
lane_filter_names = ['baseline', 'generic_straight']
# lane_filter_names = ['generic_straight']

dirn = lambda _: 'out-synthetic/%s' % _
@comptest
def test_synthetic_zero_zerophi():
    d = 0
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_zerophi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                   image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_pos_zerophi():
    d = 0.05
    phi = np.deg2rad(0)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_pos_zerophi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_neg_posphi():
    d = -0.05
    phi = np.deg2rad(15)
    
    for lane_filter_name in lane_filter_names:
        outd = dirn('test_synthetic_neg_posphi-' + lane_filter_name) 
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)
@comptest
def test_synthetic_zero_posphi():
    d = 0
    phi = np.deg2rad(15)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_posphi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

@comptest
def test_synthetic_zero_negphi():
    d = 0
    phi = np.deg2rad(-20)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_negphi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

@comptest
def test_synthetic_zero_bignegphi():
    d = 0
    phi = np.deg2rad(-50)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_bignegphi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

@comptest
def test_synthetic_zero_bigposphi():
    d = 0
    phi = np.deg2rad(+50)
    for lane_filter_name in lane_filter_names:    
        outd = dirn('test_synthetic_zero_bignegphi-' + lane_filter_name)
        test_synthetic(template, robot_name, line_detector_name,
                       image_prep_name, lane_filter_name, d, phi , outd)

     
def test_synthetic(template,robot_name,line_detector_name,
                   image_prep_name, lane_filter_name, d, phi, outd ):
    # phi = np.deg2rad(-15)
    # first, load calibration for robot
    easy_algo_db = get_easy_algo_db()
    localization_template = easy_algo_db.create_instance(FAMILY_LOC_TEMPLATES, template)
    
    gp = GroundProjection(robot_name)
    gpg = gp.gpc # GroundProjectionGeometry
    
    location = np.zeros((), dtype=TemplateStraightLane.DATATYPE_COORDS)
    location['phi'] = phi 
    location['d'] = d 
    
    xytheta = localization_template.xytheta_from_coords(location)
    sm_orig = localization_template.get_map()
    distorted_synthetic = simulate_image(sm_orig, xytheta, gpg, blur_sigma=0.3)

    image = distorted_synthetic

    res = run_pipeline(image, gp, line_detector_name, image_prep_name, lane_filter_name,
                       all_details=False,
                       skip_instagram=True, ground_truth=xytheta)

    res = dtu.resize_small_images(res)
    
    dtu.write_bgr_images_as_jpgs(res, outd, extra_string=outd.split('/')[-1])


@dtu.contract(gpg=GroundProjectionGeometry)
def simulate_image(sm_orig, xytheta, gpg, blur_sigma):
    camera_info = gpg.ci # CameraInfo
    blank = generate_blank(camera_info)
    tinfo = TransformationsInfo()
    g = dtu.SE2_from_xyth(xytheta)
    tinfo.add_transformation(frame1=FRAME_TILE, frame2=FRAME_AXLE, g=g) 
        
    sm_axle = tinfo.transform_map_to_frame(sm_orig, FRAME_AXLE)
    
    rectified_synthetic = plot_map(blank, sm_axle, gpg, do_segments=False)
    rectified_synthetic_s = cv2.GaussianBlur(rectified_synthetic, (0,0), blur_sigma)
    distorted_synthetic = gpg.distort(rectified_synthetic_s)
    distorted_synthetic = cv2.medianBlur(distorted_synthetic, 3)
    return distorted_synthetic
    
def generate_blank(camera_info):
    H = camera_info.height
    W = camera_info.width
    bgr = np.zeros(dtype='uint8', shape=(H, W, 3))
    bgr.fill(128)
    return bgr
    
if __name__ == '__main__':
    run_module_tests()