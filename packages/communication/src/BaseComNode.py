import numpy as np
from UtilStates import IntersectionType, ActionState, TrafficLightState, StopSignState
from random import choice
from PointBuffer import PointBuffer
import cv2
from time import time


class BaseComNode:
    """
    Base class for the communication node.
    This class is used to define behaviors for the node that do not directly interact with ROS. This makes testing
    easier
    """
    
    # TODO move these constants somewhere else as params?
    
    TIME_OUT_SEC = 10*60 # Duration after which the node times out and a time_out flag is published. TODO 10min for the moment
    # Permitted frequencies that can be used for flashing. We don't use ]5..10[ range since it is too close
    # to TL (Helps with background stop intersections interference)
    PERMITTED_FREQ = [2, 4, 5, 10, 15] # [2, 4, 5, 6, 8, 10, 15]
    TL_GREEN_BLINK_FREQ = 7.8 # TODO use from yaml file of TL node?
                              # https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/traffic_light/config/traffic_light_node/TL_protocol.yaml
    TL_TOLERANCE_FREQ_DIFF = 1 # 1Hz of tolerance +/-
    MAX_TRAFFIC_LIGHT_HEIGHT = 200 # Coord starts from top of image. We only account for a partial portion of the top of the image.
                                   # TODO : make it in % instead of absolute pixel 
    SOLVING_STOP_LED_COLOR = "white"
    SOLVING_TL_LED_COLOR = "red"
    SIGNALING_TOGO_DURATION = 2 # Duration in sec of "SignalingToGo" (Solid Green)
    
    

    def __init__(self, buffer_length=60, buffer_forget_time=40):
        # buffer used to keep active points
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time
        self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        
        # Current state of the node. By default, it starts as soving and intersection type Unknown
        self.curr_intersection_type = IntersectionType.Unknown
        self.curr_action_state = ActionState.Solving

        self.current_blink_freq = 0 # Init as not blinking
        time_now = time()
        self.begin_blink_time = time_now
        self.last_state_transition_time = time_now
        self.begin_solving_time_sec = time_now

        # TODO Debugging counter for tests
        self.DEBUG_COUNT = 0

    def img_callback(self, data):
        """
        This method redirect the image data to the right callback function depending on the current node state

        :param data: raw image data given by ROS
        """

        # TODO Debugging camera in
        if self.DEBUG_COUNT >= 30*10: # 30fps 10 sec
            print("img_callback")
            #print(data.header)
            self.DEBUG_COUNT = 0
        self.DEBUG_COUNT += 1

        if self.curr_intersection_type is IntersectionType.Unknown:
            return
        elif self.curr_intersection_type is IntersectionType.StopSign:
            self.stop_sign_img_callback(data)
        elif self.curr_intersection_type is IntersectionType.TrafficLight:
            self.traffic_light_img_callback(data)

    def intersection_type_callback(self, data):
        """
        This method changes the internal intersection type and initiates / terminates the different values needed

        :param data: intersection data given by ROS
        """
        new_intersection_type = IntersectionType(data)

        # TODO: fill the state machine for transitions
        if self.curr_intersection_type != new_intersection_type:
    
            # Reset all time_trackers since a new intersection means a new cycle
            self.begin_solving_time_sec = time()
            self.last_state_transition_time = time()

            # Reinitialize the img_buffer
            self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
        
            # Stop sign
            if new_intersection_type is IntersectionType.StopSign:
                self.update_action_state(ActionState.Solving)
                # Choose a bink freq and reset blink time tracker
                # TODO perhaps all of the following should be done in run()?
                self.current_blink_freq = choice(self.PERMITTED_FREQ)
                self.blink_at(self.current_blink_freq, self.SOLVING_STOP_LED_COLOR)
                self.begin_blink_time = time()
            
            elif new_intersection_type is IntersectionType.TrafficLight:
                self.update_action_state(ActionState.Solving)
                # Stop blinking and set solid color to "red"
                self.current_blink_freq = 0
                # TODO make sure to do this in only one place
#                self.blink_at(self.current_blink_freq, self.SOLVING_TL_LED_COLOR)

            self.curr_intersection_type = new_intersection_type

    # Elapsed time in current state (since last transition)
    def elapsed_time_in_curr_state(self):
        return time() - self.last_state_transition_time

    # Verify if the point we have is in the TL green frequency range.
    def is_in_tl_frequency_range(self, point_frequency):
        min_freq = self.TL_GREEN_BLINK_FREQ - self.TL_TOLERANCE_FREQ_DIFF
        max_freq = self.TL_GREEN_BLINK_FREQ + self.TL_TOLERANCE_FREQ_DIFF
        if point_frequency >= min_freq and point_frequency <= max_freq:
            return True
        return False

    def blink_at(self, frequency: int):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param frequency: frequency of blinking
        """
        pass  # placeholder for inheritance

    # All state updates should be done here
    def update_action_state(self, new_action_state):

        # Verify if action sate has changed
        if self.curr_action_state != new_action_state:
            print(f"Transition from {self.curr_action_state} to {new_action_state}")
            self.curr_action_state = new_action_state
            self.last_state_transition_time = time()

            # Reinitialize the img_buffer when we switch state to Solving
            if new_action_state == ActionState.Solving:
                self.buffer = PointBuffer(self.buffer_length, self.buffer_forget_time)
            

    def stop_sign_img_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the intersection
        type is ``IntersectionType.StopSign``. It then adds this image to the buffer

        :param data: raw image data given by ROS
        """
        # TODO: Use a different self.buffer than the one used for TrafficLight?
        #       Here we might split the image to 3 parts (Left, Middle and Right) to detect each bot separately...
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.buffer.push_frame(new_img)


    def traffic_light_img_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the intersection
        type is  ``IntersectionType.TrafficLight``.

        :param data: raw image data given by ROS
        """
        # TODO: use a different self.buffer than the one used for StopSign?
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.buffer.push_frame(new_img)

    
    # Mehtod to handle the transition from the state "SignalingToGo" to the stat "Go"
    def handle_togo_transition(self):
        # Stay in this SignalingToGo state for a certain duration.
        if (self.curr_action_state == ActionState.SignalingToGo
            and self.elapsed_time_in_curr_state() > self.SIGNALING_TOGO_DURATION):
                self.update_action_state(ActionState.Go)

    def run(self):
        """
        Method that surveys the real world state and determines what to do

        :return: the go or wait signal
        """

        # TODO: fill

        current_time_sec = time()

        if self.curr_intersection_type == IntersectionType.TrafficLight:

            if self.curr_action_state == ActionState.Solving:    
                # Once we detect at least 1 "light" point flashing we assume it is the traffic light
                if len(self.buffer.points) > 0:
                    # Check if the flashing frequnecy is close enough to the TL (7.8Hz)
                    # We go through all the detected points. Since we might have captured others from background
                    for tl_led_point in self.buffer.points:
                        # TODO perhaps it is better to just crop the image when received instead of analysing the full image
                        #if (tl_led_point.coords[0] <= self.MAX_TRAFFIC_LIGHT_HEIGHT # coord starts from top of image
                        #    and self.is_in_tl_frequency_range(tl_led_point.get_frequency()[0])):
                        if (self.is_in_tl_frequency_range(tl_led_point.get_frequency()[0])):
                            print(f"Freq : {tl_led_point.get_frequency()[0]}")
                            self.update_action_state(ActionState.SignalingToGo)
                            break
            # Final step from state "SignalingToGo" to the stat "Go"
            self.handle_togo_transition()
            

        elif self.curr_intersection_type == IntersectionType.StopSign:
            # TODO
            # ...

            # TODO Add an advanced logic to account for elapsed time in stop sign negotiation
            # It might help in case the bot has been waiting for too long. Use the elapsed_time_in_curr_state()
            # ...
            
            # Final step from state "SignalingToGo" to the stat "Go"
            self.handle_togo_transition()

        elif self.curr_intersection_type == IntersectionType.Unknown:
            # TODO
            pass
        else:
            # TODO should never happen?
            pass


        # Update last_time
        self.last_time_sec = current_time_sec

        # Check for timeout based on the self.begin_solving_time_sec and TIME_OUT_SEC
        if self.last_time_sec - self.begin_solving_time_sec > self.TIME_OUT_SEC:
            self.update_action_state(ActionState.TimedOut)
        
        #for i, point in enumerate(self.buffer.points):
        #    print(f"{i} -- freq: {point.get_frequency()[0]} -- {point}")
        #print("BaseComNode->run() call")

