import numpy as np
from random import choice
from time import time
from PointBuffer import PointBuffer
from UtilStates import StopSignState


class StopSignSolver:
    """
    Class implementing the logic for solving Stop Sign intersection
    """

    # Permitted frequencies that can be used for flashing. We don't use ]5..10[ range since it is too close
    # to TL (Helps with background stop intersections interference)
    PERMITTED_FREQ = [2, 4, 5, 10, 15] # [2, 4, 5, 6, 8, 10, 15]
    # Color of LED when in SS solving (blinking of course)
    SOLVING_STOP_LED_COLOR = "white"
    # Sensing duration (in sec) when the solver is started. The longer it is the more certain we are about other bots
    # but it also needs to be relatively short to allow for quick decision making.
    INITIAL_SENSING_DURATION_SEC = 20
    # These parameters determine the random interval from which we need to choose a wait time in sec
    # when we have the same conflicting blinking frequency as other bots. The value should be chosen randomly
    # from this interval to make sure we don't end up in a same frequency infinite condition
    MIN_MAX_CONFLICT_WAIT_SAME_FREQ_SEC = [5, 15]


    def __init__(self, buffer_length, buffer_forget_time):
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time
        self.right_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.center_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.right_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.ss_state = StopSignState.Sensing
        self.begin_sensing_time = time() # time we stated sensing

        # State and information about the other duckiebots
        self.right_bot_freq = None
        self.center_bot_freq = None
        self.left_bot_freq = None

        # Initial blinking frequency
        self.current_blink_freq = 0
        self.begin_blink_time = time() # need to compute for how long the same freq is used
        self.select_blinkfreq()

        # Duration for initial sensing
        self.initial_sensing_duration_sec = self.INITIAL_SENSING_DURATION_SEC

        # Wait time when conflicting freq is detected with other bots
        self.conflict_wait_with_same_freq_sec = None


    # Resets the solver to it's initial state. Should be called when ever the solving should be restarted
    # for example when the intersection type is received/change
    def reset(self):
        print("StopSignSolver->reset()")
        self.right_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.center_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.right_bot_imgbuffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.update_ss_state(StopSignState.Sensing)
        self.begin_sensing_time = time() # time we stated sensing
        
        # Rest state and information about the other duckiebots
        self.right_bot_freq = None
        self.center_bot_freq = None
        self.left_bot_freq = None

        # Reset blinking frequency
        self.current_blink_freq = 0
        self.begin_blink_time = time() # need to compute for how long the same freq is used
        self.select_blinkfreq()

        # Wait time when conflicting freq is detected with other bots
        self.conflict_wait_with_same_freq_sec = None


    # Use this to reset/init blinking frequency
    def select_blinkfreq(self):
        self.current_blink_freq = choice(self.PERMITTED_FREQ)
        # reset the beginning of blink time
        self.begin_blink_time = time()


    # Choose a wait time in sec when we have the same conflicting blinking frequency as other bots
    def choose_conflict_wait_with_same_freq(self):
        min_wait = self.MIN_MAX_CONFLICT_WAIT_SAME_FREQ_SEC[0]
        max_wait = self.MIN_MAX_CONFLICT_WAIT_SAME_FREQ_SEC[0]
        interval = max_wait - min_wait
        self.conflict_wait_with_same_freq_sec = min_wait + interval*np.random.random()


    # All state updates should be done here
    def update_ss_state(self, new_ss_state):
        # Verify if sate has changed
        if self.ss_state != new_ss_state:
            print(f"StopSignSolver: Transition from {self.ss_state} to {new_ss_state}")
            self.ss_state = new_ss_state


    # Return blinking frequency and color of LED when running this solver
    def get_blinkfreq_and_color(self):
        return self.current_blink_freq, self.SOLVING_STOP_LED_COLOR


    # Accumulate the camera images
    def push_camera_image(self, new_img):
        # TODO: Use a different self.buffer fore each duckiebot? Left, Front, Right?
        #       We might split the image to 3 parts (Left, Middle and Right) to detect each bot separately...
        if self.ss_state == StopSignState.Sensing: # Similar to TL, use images only when Sensing, but not sure yet
            pass


    def is_clear_to_go(self):
        # TODO other checks?
        return self.ss_state == StopSignState.HasHighestPriority


    # Step call of the solver to run the analysis on the accumulated images from the camera
    # This method is to be called by the Node run() method continuously
    def step_solver(self):
        # TODO
        # Some of the processing steps that needs to be implemented
        #  
        # Read other bots frequencies
        # use the 3 imgbuffers self.right_bot_imgbuffer...
        # ...

        # Make sure self has a different blinking frequency compared to others
        # ...

        # Establish priority of all bots (including self)
        # ...

        # Adjust blinking freq if required (Other bot(s) has same freq since X time)
        # Use choose_conflict_wait_with_same_freq() to select a random waiting time. Should be
        # called once with in the same conflicting state.
        # State should be switched to ConflictingBlinkFreq...
        # Use self.begin_blink_time to compare to...
        # Use select_blinkfreq() to select a new freq if needed (wait time exhausted)
        # ...

        pass


