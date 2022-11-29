import numpy as np
from PointBuffer import PointBuffer
from UtilStates import TrafficLightState


class TrafficLightSolver:
    """
    Class implementing the logic for solving Traffic Light intersection
    """

    TL_GREEN_BLINK_FREQ = 7.8 # TODO use from yaml file of TL node?
                              # https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/traffic_light/config/traffic_light_node/TL_protocol.yaml
    TL_TOLERANCE_FREQ_DIFF = 1 # 1Hz of tolerance +/-
    MAX_TRAFFIC_LIGHT_HEIGHT = 200 # Coord starts from top of image. We only account for a partial portion of the top of the image.
                                   # TODO : make it in % instead of absolute pixel 
    SOLVING_TL_LED_COLOR = "red"
    SOLVING_TL_FREQ = 0

    def __init__(self, buffer_length, buffer_forget_time):
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time
        self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.tl_state = TrafficLightState.Sensing

    # Verify if the point we have is in the TL green frequency range.
    def is_in_tl_frequency_range(self, point_frequency):
        min_freq = self.TL_GREEN_BLINK_FREQ - self.TL_TOLERANCE_FREQ_DIFF
        max_freq = self.TL_GREEN_BLINK_FREQ + self.TL_TOLERANCE_FREQ_DIFF
        if point_frequency >= min_freq and point_frequency <= max_freq:
            return True
        return False

    # Accumulate the camera images
    def push_camera_image(self, new_img):
        # Only post image when state is Sensing. No need in other states
        if self.tl_state == TrafficLightState.Sensing:
            # TODO perhaps it is better to just crop the image when received instead of analyzing the full image
            reduced_img = new_img[:self.MAX_TRAFFIC_LIGHT_HEIGHT+1,:].copy()
            self.buffer.push_frame(reduced_img)

    # Resets the solver to it's initial state. Should be called when ever the solving should be restarted
    # for example when the intersection type is received/change
    def reset(self):
        print("TrafficLightSolver->reset()")
        self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        self.update_tl_state(TrafficLightState.Sensing)

    def should_go(self):
        # TODO other checks?
        return self.tl_state == TrafficLightState.Green

    # All state updates should be done here
    def update_tl_state(self, new_tl_state):
        # Verify if sate has changed
        if self.tl_state != new_tl_state:
            print(f"TrafficLightSolver: Transition from {self.tl_state} to {new_tl_state}")
            self.tl_state = new_tl_state

    # Return blinking frequency and color of LED when running this solver
    def get_blinkfreq_and_color(self):
        return self.SOLVING_TL_FREQ, self.SOLVING_TL_LED_COLOR

    # Step call of the solver to run the analysis on the accumulated images from the camera
    # This method is to be called by the Node run() method continuously
    def step_solver(self):
        # Once we detect at least 1 "light" point flashing we assume it is the traffic light and 
        # traffic light is still "red"
        if len(self.buffer.points) > 0 and self.tl_state == TrafficLightState.Sensing:
            # Check if the flashing frequency is close enough to the TL (7.8Hz)
            # We go through all the detected points. Since we might have captured others from background
            for tl_led_point in self.buffer.points:
                # TODO perhaps it is better to just crop the image when received instead of analyzing the full image
                if (tl_led_point.coords[0] <= self.MAX_TRAFFIC_LIGHT_HEIGHT # coord starts from top of image
                    and self.is_in_tl_frequency_range(tl_led_point.get_frequency()[0])):
                    print(f"Freq : {tl_led_point.get_frequency()[0]}")
                    self.update_tl_state(TrafficLightState.Green)
                    break # Break right away, we just detected the TL Green state

        # Return the tl green status
        return self.is_clear_to_go()


