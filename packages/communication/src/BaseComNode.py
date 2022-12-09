import numpy as np
from UtilStates import IntersectionType, ActionState
import cv2
from time import time
from stop_sign_solver import StopSignSolver
from traffic_light_solver import TrafficLightSolver
from duckietown_msgs.msg import TagInfo


class BaseComNode:
    """
    Base class for the communication node.
    This class is used to define behaviors for the node that do not directly interact with ROS. This makes testing
    easier
    """
    # TODO move these constants somewhere else as params?
    TIME_OUT_SEC = 2*60 # Duration after which the node times out and a time_out flag is published. TODO 10min for the moment

    def __init__(self, buffer_length=60, buffer_forget_time=40):
        # buffers parameters used to keep active points
        self.buffer_length = buffer_length
        self.buffer_forget_time = buffer_forget_time

        # Define TL and SS solvers
        self.tl_solver = TrafficLightSolver(buffer_length, buffer_forget_time)
        self.ss_solver = StopSignSolver(buffer_length, buffer_forget_time)

        # Current state of the node. By default, it starts as Unknown and intersection type Unknown
        self.curr_intersection_type = IntersectionType.Unknown

        time_now = time()
        self.last_state_transition_time = time_now
        self.begin_solving_time_sec = time_now

    def img_callback(self, data):
        """
        This method redirect the image data to the right callback function depending on the current node state

        :param data: raw image data given by ROS
        """
        if self.curr_intersection_type is IntersectionType.Unknown:
            return
        elif self.curr_intersection_type is IntersectionType.StopSign:
            self.stop_sign_img_callback(data)
        elif self.curr_intersection_type is IntersectionType.TrafficLight:
            self.traffic_light_img_callback(data)

    # CALLBACK SECTION
    def intersection_type_callback(self, msgs):
        """
        This method changes the internal intersection type and initiates / terminates the different values needed

        :param data: intersection data given by ROS
        """
        # new_intersection_type = IntersectionType(data.infos)
        tag_info = TagInfo()
        new_intersection_type = IntersectionType.Unknown
        for info in msgs.infos:

            if(info.tag_type == tag_info.SIGN):
                 if(info.traffic_sign_type == tag_info.STOP):
                     new_intersection_type = IntersectionType.StopSign
                 elif(info.traffic_sign_type == tag_info.T_LIGHT_AHEAD):
                     new_intersection_type = IntersectionType.TrafficLight


        if self.curr_intersection_type == new_intersection_type:
            return

        # TODO: fill the state machine for transitions
        # Reset all time_trackers since a new intersection means a new cycle
        self.begin_solving_time_sec = time()
        self.last_state_transition_time = time()

        # Stop sign
        if new_intersection_type is IntersectionType.StopSign:
            self.ss_solver.reset()
            self.blink_at(self.ss_solver.blink_freq)

        elif new_intersection_type is IntersectionType.TrafficLight:
            self.tl_solver.reset()

        self.curr_intersection_type = new_intersection_type

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

    # RUN SECTION
    def run(self):
        """
        Method that surveys the real world state and determines what to do

        :return: the go or wait signal
        """
        action = ActionState.Solving
        #print(self.curr_intersection_type)
        if self.curr_intersection_type is IntersectionType.Unknown:
            return
        if self.curr_intersection_type is IntersectionType.TrafficLight:
            action = self.traffic_light_run()
        elif self.curr_intersection_type is IntersectionType.StopSign:
            action = self.stop_sign_run()
        self.update_action_state(action)

        # Check for timeout based on the self.begin_solving_time_sec and TIME_OUT_SEC
        if time() - self.begin_solving_time_sec > self.TIME_OUT_SEC:
            self.update_action_state(ActionState.TimedOut)

    def traffic_light_run(self):
        # Step the solver
        self.tl_solver.step_solver()
        if self.tl_solver.should_go():
            return ActionState.Go
        return ActionState.Solving

    def stop_sign_run(self):
        if self.ss_solver.begin_blink_time + self.ss_solver.SENSING_DURATION <= time():
            if self.ss_solver.should_go():
                self.blink_at(0)
                return ActionState.Go

            self.ss_solver.reset()
            self.blink_at(self.ss_solver.blink_freq)

        print('points', len(self.ss_solver.point_buffer.points))
        buffer = self.ss_solver.point_buffer.points
        for i, point in enumerate(buffer):
            freq, spikes = point.get_frequency()
            print(f"{i} -- {freq}: {point}\n{i} -- {spikes}")
        return ActionState.Solving

    # OVERWRITTEN SECTION
    def blink_at(self, frequency: int, color: str='white'):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param frequency: frequency of blinking
        """
        print('self-freq:', frequency)
        pass  # placeholder for inheritance

    # OVERWRITTEN SECTION
    def publish_signal(self, action: ActionState):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param action: the go signal
        """
        print('Action:', action)
        pass  # placeholder for inheritance

    # INTERNAL SECTION
    def update_action_state(self, action_state):
        """
        All state updates should be done here
        """
        if action_state in [ActionState.Go, ActionState.TimedOut]:
            self.intersection_type_callback(IntersectionType.Unknown)
            self.publish_signal(action_state)
