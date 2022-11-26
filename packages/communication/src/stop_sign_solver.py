import numpy as np
from PointBuffer import PointBuffer
from UtilStates import StopSignState


class StopSignSolver:
    """
    Class implementing the logic for solving Stop Sign intersection
    """
    def __init__(self, buffer_length, buffer_forget_time):
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time
        self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.ss_state = StopSignState.Sensing

    # Accumulate the camera images
    def push_camera_image(self, new_img):
        # TODO: Use a different self.buffer fore each duckiebot? Left, Front, Right?
        #       We might split the image to 3 parts (Left, Middle and Right) to detect each bot separately...
        pass

   # Resets the solver to it's initial state. Should be called when ever the solving should be restarted
    # for example when the intersection type is received/change
    def reset(self):
        print("StopSignSolver->reset()")
        self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.ss_state = StopSignState.Sensing

    def is_clear_to_go(self):
        # TODO other checks?
        return self.ss_state == StopSignState.HasHighestPriority

    # Step call of the solver to run the analysis on the accumulated images from the camera
    # This method is to be called by the Node run() method continuously
    def step_solver(self):
        # TODO
        pass


