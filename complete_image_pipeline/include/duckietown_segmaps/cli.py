from duckietown_utils.cli import D8App
from easy_algo.algo_db import get_easy_algo_db
from duckietown_segmaps.maps import FAMILY_SEGMAPS, _plot_map_segments
from duckietown_utils.matplotlib_utils import CreateImageFromPylab
import duckietown_utils as dtu
import os
from reprep.plot_utils.axes import turn_all_axes_off
from duckietown_utils.file_utils import write_data_to_file
from geometry.poses import SE2_from_translation_angle
from complete_image_pipeline_tests.synthetic import simulate_image
from ground_projection.ground_projection_interface import GroundProjection

class DisplayTileAndMaps(D8App):
    """ 
          
    """

    def define_program_options(self, params):
        pass
    
#         params.add_string('output', default=None, short='-o', help='Output directory', group=g) 
    
    def go(self):

        maps = []
        
#         maps += ['DT17_four_way']
#         maps += ['DT17_curve_right']
#         maps += ['DT17_map_loop3']
        maps += ['DT17_before_curve']
        out = 'out-maps'
        
        db = get_easy_algo_db()
        maps = list(db.query_and_instance(FAMILY_SEGMAPS, '*'))
        
        maps = ['DT17_curve_left']
        print('maps: %s' % maps)
        for id_map in maps:
            display_map(id_map, out)
            
def display_map(id_map, out):
    dtu.logger.info('id_map == %s' % id_map)
    db = get_easy_algo_db()
    smap = db.create_instance(FAMILY_SEGMAPS, id_map)
    texture_png = get_texture(smap, dpi=600)
    fn = os.path.join(out, '%s-texture.png' % (id_map))
    write_data_to_file(texture_png, fn)
    
    bgr = simulate_camera_view(smap)
    fn = os.path.join(out, '%s-view.jpg' % (id_map))
    dtu.write_bgr_to_file_as_jpg(bgr, fn)

def get_texture(smap, dpi):
    figure_args=dict(figsize=(2,2), facecolor='green')
    a = CreateImageFromPylab(dpi=dpi, figure_args=figure_args)
    frames = list(set(_.id_frame for _ in smap.points.values()))
    id_frame = frames[0]
#     print('frames: %s choose %s' % (frames, id_frame))
    with a as pylab:
        _plot_map_segments(smap, pylab, id_frame, plot_ref_segments=False)
        pylab.axis('equal')
        turn_all_axes_off(pylab)
        pylab.tight_layout()
    png = a.get_png()
    return png

def simulate_camera_view(sm):
    robot_name = 'flitzer'
    gp = GroundProjection(robot_name)
    # GroundProjectionGeometry
    gpg = gp.get_ground_projection_geometry() 
    
    pose = SE2_from_translation_angle([0,-0.05],-np.deg2rad(-5))
    distorted_synthetic = simulate_image(sm, pose, gpg, blur_sigma=0.3)
    return distorted_synthetic

        
    
import numpy as np