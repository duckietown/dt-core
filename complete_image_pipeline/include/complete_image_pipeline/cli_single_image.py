from duckietown_utils.cli import D8App
from duckietown_utils.image_writing import write_jpgs_to_dir
from duckietown_utils.download import get_file_from_url
from duckietown_utils.jpg import image_cv_from_jpg_fn
from complete_image_pipeline.pipeline import run_pipeline
from duckietown_utils.test_hash import get_md5


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
            image_filename = get_file_from_url(image_filename)

        image_cv = image_cv_from_jpg_fn(image_filename)
    
        res  = run_pipeline(image_cv, line_detector, image_prep)
        
        output = self.options.output
        if output is None:
            output = 'out-pipeline-' + get_md5(self.options.image)[:6]
            self.info('No --output given, using %s' % output)
            
        write_jpgs_to_dir(res, output)
    
