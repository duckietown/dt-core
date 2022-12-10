from random import choice
from time import time
from threading import Lock
from PointBuffer import PointBuffer


class StopSignSolver:
    """
    Class implementing the logic for solving Stop Sign intersection
    """

    # Permitted frequencies that can be used for flashing (READABLE: [2, 4, 5, 6, 8, 10, 15])
    PERMITTED_FREQ = [2, 4, 5, 8, 10, 15]
    # Sensing duration (in sec) for a sensing step
    SENSING_DURATION = 5

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

    def reset(self):
        """
        Resets the solver to it's initial state. Should be called when ever the solving should be restarted
        for example when the intersection type is received/change
        """
        with self.buffer_lock:
            self.point_buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)

        # Reset blinking frequency
        self.blink_freq = choice(self.PERMITTED_FREQ)
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
                if point.get_frequency()[0] >= self.blink_freq:
                    return False
            return True
