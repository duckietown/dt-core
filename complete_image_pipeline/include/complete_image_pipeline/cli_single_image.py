from duckietown_utils.cli import D8App
import duckietown_utils as dtu
from .pipeline import run_pipeline

__all__ = [
    'SingleImagePipeline',
]

class SingleImagePipeline(D8App):
    """ 
        Runs the vision pipeline on a single image.  
    """

    def define_program_options(self, params):
        g = "Input/output"
        params.add_string('image', help="Image to use.", group=g)
        params.add_string('output', default=None, short='-o', help='Output directory', group=g) 
        g = "Pipeline"
        params.add_string('line_detector', default='baseline', help="Which line detector to use", group=g)
        params.add_string('image_prep', default='prep_200_100', help="Which image prep to use", group=g)
        
    
    def go(self):
        line_detector = self.options.line_detector
        image_prep = self.options.image_prep
        
        self.info('Line detector: %s' % line_detector)
        self.info('image_prep: %s' % image_prep)
  
        image_filename = self.options.image
        if image_filename.startswith('http'):
            image_filename = dtu.get_file_from_url(image_filename)

        image_cv = dtu.image_cv_from_jpg_fn(image_filename)
    
        res  = run_pipeline(image_cv, line_detector, image_prep)
        
        output = self.options.output
        if output is None:
            output = 'out-pipeline-' + dtu.get_md5(self.options.image)[:6]
            self.info('No --output given, using %s' % output)
            
        dtu.write_jpgs_to_dir(res, output)
    
