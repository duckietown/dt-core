from random import choice
from time import time
from threading import Lock
from PointBuffer import PointBuffer


class StopSignSolver:
    """
    Class implementing the logic for solving Stop Sign intersection
    """

    # Permitted frequencies that can be used for flashing (READABLE: [2, 4, 5, 6, 8, 10, 15])
    # TODO: Based on tests:
    #  1 is not readable
    #  7 and 8 are too fast to be emitted consistently (Perhaps with some adjustment of the fifo update rate it gets better?)
    #  For now Best are 2, 3, 4, 5 and 6. We use [2, 4, 6] to keep a margin.
    PERMITTED_FREQ = [2, 4, 6] #[2, 4, 5, 8, 10, 15]
    FREQ_ERROR_UPPER_MARGIN = 1 # This is used (Added to the read freq) as an upper error margin to account for reading higher freq
                                # than what the other bot is really flashing at. It avoids mistakenly deciding a GO. We prefer it
                                # to wait for another cycle in this case.

    # Sensing duration (in sec) for a sensing step
    SENSING_DURATION = 10

    DEFAULT_IMG_FPS = 30

    def __init__(self, buffer_length, buffer_forget_time):
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time

        self.point_buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.buffer_lock = Lock()

        # Initial blinking frequency
        self.blink_freq = 0
        self.begin_blink_time: float = 0  # need to compute for how long the same freq is used
        self.reset()

        # Duration for initial sensing
        self.sensing_duration = self.SENSING_DURATION

        # Wait time when conflicting freq is detected with other bots
        self.conflict_wait_with_same_freq_sec = None

        self.img_avrg_fps = self.DEFAULT_IMG_FPS # Start with default FPS

    def update_fps(self, fps):
        self.img_avrg_fps = fps

    def reset(self):
        """
        Resets the solver to it's initial state. Should be called when ever the solving should be restarted
        for example when the intersection type is received/change
        """
        with self.buffer_lock:
            self.point_buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)

        # Reset blinking frequency
        print(f" reset, OLD self-freq: {self.blink_freq}")
        self.blink_freq = choice(self.PERMITTED_FREQ)
        print(f" reset, NEW self-freq: {self.blink_freq}")
        self.begin_blink_time = time()

    def get_blink_freq(self):
        """
        Return blinking frequency and color of LED when running this solver
        """
        return self.blink_freq

    def push_camera_image(self, new_img):
        """
        Accumulate the camera images
        """
        with self.buffer_lock:
            self.point_buffer.push_frame(new_img)

    def should_go(self):
        with self.buffer_lock:
            for point in self.point_buffer.points:

                # TODO Tests compare 30 fps to computed img_avrg_fps
                freq_30pfs = point.get_frequency()[0]
                freq = point.get_frequency(self.img_avrg_fps)[0]
                fps = max(self.img_avrg_fps, self.DEFAULT_IMG_FPS)
                freq_to_use = point.get_frequency(fps)[0]
                print(f"  ***** OTHERS freq: {freq}, OTHERS freq_30pfs: {freq_30pfs}, OTHERS freq_to_use: {freq_to_use}, SELF freq: {self.blink_freq}, IMG AVG FPS: {self.img_avrg_fps}")

                # TODO Based on some testing with the bots, when the computed FPS (self.img_avrg_fps) is > 30 we get good accurate
                # results to the real frequencies passing the self.img_avrg_fps to point.get_frequency(). However, for some strange reason
                # when the computed FPS is way to low (like 18 FPS) both methods (default 30 vs computed fps) are not accurate but using
                # default 30 FPS get's closer results to the true frequencies.
                # -> Use max (self.img_avrg_fps, 30).
                fps = max(self.img_avrg_fps, self.DEFAULT_IMG_FPS)
                freq = point.get_frequency(fps)[0]

                if freq + self.FREQ_ERROR_UPPER_MARGIN >= self.blink_freq:
                    return False
            return True
