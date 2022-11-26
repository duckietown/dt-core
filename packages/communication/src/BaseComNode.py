import numpy as np
from UtilStates import IntersectionType, ActionState, TrafficLightState, StopSignState
import cv2
from time import time
from stop_sign_solver import StopSignSolver
from traffic_light_solver import TrafficLightSolver


class BaseComNode:
    """
    Base class for the communication node.
    This class is used to define behaviors for the node that do not directly interact with ROS. This makes testing
    easier
    """
    
    # TODO move these constants somewhere else as params?
    TIME_OUT_SEC = 10*60 # Duration after which the node times out and a time_out flag is published. TODO 10min for the moment
    SIGNALING_TOGO_DURATION = 2 # Duration in sec of "SignalingToGo" (Solid Green)


    def __init__(self, buffer_length=60, buffer_forget_time=40):
        # buffers parameters used to keep active points
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time

        # Define TL and SS solvers
        self.tl_solver = TrafficLightSolver(buffer_length, buffer_forget_time)
        self.ss_solver = StopSignSolver(buffer_length, buffer_forget_time)
        
        # Current state of the node. By default, it starts as Unknown and intersection type Unknown
        self.curr_intersection_type = IntersectionType.Unknown
        self.curr_action_state = ActionState.Solving
        self.current_blink_freq = 0 # Init as not blinking
        time_now = time()
        self.last_state_transition_time = time_now
        self.begin_solving_time_sec = time_now

        # TODO Debugging counter for tests
        self.DEBUG_COUNT = 0


    # Reset method to reset everything to initial state
    # Should be used once a "GO" is published just to be ready for next round and avoid undefined states/behaviors
    def reset(self):
        print("BaseComNode->reset()")
        self.tl_solver.reset()
        self.ss_solver.reset()

        # Current state of the node. By default, it starts as Unknown and intersection type Unknown
        self.curr_intersection_type = IntersectionType.Unknown
        self.update_action_state(ActionState.Solving)
        self.current_blink_freq = 0 # Init as not blinking
        time_now = time()
        self.last_state_transition_time = time_now
        self.begin_solving_time_sec = time_now


    def img_callback(self, data):
        """
        This method redirect the image data to the right callback function depending on the current node state

        :param data: raw image data given by ROS
        """

        # TODO Debugging camera in
        if self.DEBUG_COUNT >= 30*10: # print every 30fps 10 sec
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

            # Stop sign
            if new_intersection_type is IntersectionType.StopSign:
                self.ss_solver.reset()
                self.update_action_state(ActionState.Solving)
                # Choose a blink freq and reset blink time tracker
                self.current_blink_freq, self.current_color = self.ss_solver.get_blinkfreq_and_color()
                self.blink_at(self.current_blink_freq, self.current_color)
            
            elif new_intersection_type is IntersectionType.TrafficLight:
                self.tl_solver.reset()
                self.update_action_state(ActionState.Solving)
                # Stop blinking and set solid color to "red"
                self.current_blink_freq, self.current_color = self.tl_solver.get_blinkfreq_and_color()
                # TODO make sure to do this in only one place
                self.blink_at(self.current_blink_freq, self.current_color)

            self.curr_intersection_type = new_intersection_type


    # Elapsed time in current state (since last transition)
    def elapsed_time_in_curr_state(self):
        return time() - self.last_state_transition_time


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
                # Reset both TL and SS solvers
                self.tl_solver.reset()
                self.ss_solver.reset()


    def stop_sign_img_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the intersection
        type is ``IntersectionType.StopSign``. It then adds this image to the buffer

        :param data: raw image data given by ROS
        """
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.ss_solver.push_camera_image(new_img)


    def traffic_light_img_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the intersection
        type is  ``IntersectionType.TrafficLight``.

        :param data: raw image data given by ROS
        """
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.tl_solver.push_camera_image(new_img)


    # Method to handle the transition from the state "SignalingToGo" to the stat "Go"
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

        current_time_sec = time()

        if self.curr_intersection_type == IntersectionType.TrafficLight:

            if self.curr_action_state == ActionState.Solving:

                # Step the solver
                self.tl_solver.step_solver()
                if self.tl_solver.is_clear_to_go():
                    self.update_action_state(ActionState.SignalingToGo)

            # Final step from state "SignalingToGo" to the stat "Go"
            self.handle_togo_transition()
            

        elif self.curr_intersection_type == IntersectionType.StopSign:

            # TODO this goes inside the StopSignSolver()...
            # Add an advanced logic to account for elapsed time in stop sign negotiation
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
        
