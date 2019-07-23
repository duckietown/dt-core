import os

import duckietown_utils as dtu
from duckietown_utils.cli import D8App
from ground_projection import GroundProjection

from .pipeline import run_pipeline

__all__ = [
    'SingleImagePipeline',
]


class SingleImagePipeline(D8App):
    """
        Runs the vision pipeline on a single image.
    """

    cmd = 'rosrun complete_image_pipeline single_image_pipeline'

    def define_program_options(self, params):
        g = "Input/output"
        params.add_string('image', help="Image to use.", group=g, default=None)
        params.add_string('output', default=None, short='-o', help='Output directory', group=g)
        g = "Pipeline"
        params.add_string('line_detector', default='baseline', help="Which line detector to use", group=g)
        params.add_string('image_prep', default='baseline', help="Which image prep to use", group=g)
        params.add_string('anti_instagram', default='baseline', help="Which anti_instagram to use", group=g)
        params.add_string('lane_filter',
                          # default='baseline',
                           default='moregeneric_straight',
                          help="Which lane_filter to use", group=g)

    def go(self):
        output = self.options.output
        if output is None:
            output = 'out-pipeline'  #  + dtu.get_md5(self.options.image)[:6]
            self.info('No --output given, using %s' % output)

        if self.options.image is not None:
            image_filename = self.options.image
            if image_filename.startswith('http'):
                image_filename = dtu.get_file_from_url(image_filename)

            bgr = dtu.bgr_from_jpg_fn(image_filename)
        else:
            self.info('Taking a picture. Say cheese!')
            out = os.path.join(output, 'frame.jpg')
            bgr = dtu.bgr_from_raspistill(out)
            self.info('Picture taken: %s ' % str(bgr.shape))

            bgr = dtu.d8_image_resize_fit(bgr, 640)
            self.info('Resized to: %s ' % str(bgr.shape))

        vehicle_name = dtu.get_current_robot_name()
        gp = GroundProjection(vehicle_name)

        if False:
            _res = run_pipeline(bgr, gp,
                                line_detector_name=self.options.line_detector,
                                image_prep_name=self.options.image_prep,
                                anti_instagram_name=self.options.anti_instagram,
                                lane_filter_name=self.options.lane_filter)

        dtu.DuckietownConstants.show_timeit_benchmarks = True
        res, _stats = run_pipeline(bgr, gp,
                            line_detector_name=self.options.line_detector,
                            image_prep_name=self.options.image_prep,
                            anti_instagram_name=self.options.anti_instagram,
                            lane_filter_name=self.options.lane_filter)

        self.info('resizing')
        res = dtu.resize_small_images(res)
        self.info('writing')
        dtu.write_bgr_images_as_jpgs(res, output)

#        dtu.write_jpgs_to_dir(res, output)
