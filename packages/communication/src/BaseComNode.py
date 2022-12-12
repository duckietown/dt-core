import numpy as np
from UtilStates import IntersectionType, ActionState
import cv2
from time import time, sleep
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
    TIME_OUT_SEC = 2 * 60  # Duration after which the node times out and a time_out flag is published. TODO 10min for the moment

    FPS_HISTORY_DURATION_SEC = 2 # Keep 2sec of images time stamps history
    FPS_UPDATE_PERIOD_SEC = 1 # Period in sec to compute and update fps
    DEFAULT_FPS = 30

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

        # For estimating current average FPS
        self.time_stamps_array = np.array([])
        self.img_avrg_fps = self.DEFAULT_FPS # Start with default FPS
        self.last_drop_time = time()

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

        prev_fps = self.img_avrg_fps
        new_time_stamp_sec = data.header.stamp.to_sec()
        # Add current time_stamp
        self.time_stamps_array = np.append(self.time_stamps_array, new_time_stamp_sec)
        # Compute FPS
        if (time() - self.last_drop_time > self.FPS_UPDATE_PERIOD_SEC):
            self.last_drop_time = time()
            history_duration_sec = self.time_stamps_array[-1] - self.time_stamps_array[0]
            # Compute the average diff between the times stamps and FPS
            #if history_duration_sec > self.FPS_UPDATE_PERIOD_SEC:
            if len(self.time_stamps_array) > 1:
                diffs = self.time_stamps_array[1:] - self.time_stamps_array[:-1]
                self.img_avrg_fps = 1/diffs.mean()
                #print(f"History duration: {history_duration_sec}")
                #print(f"Computed FPS from time stamps: {self.img_avrg_fps}")

            # Drop the oldest time steps to keep the array with a period around self.FPS_HISTORY_DURATION_SEC
            if history_duration_sec > self.FPS_HISTORY_DURATION_SEC:
                num_samples_tokeep = int(np.floor(self.img_avrg_fps * self.FPS_HISTORY_DURATION_SEC))
                number_of_samples_to_drop = len(self.time_stamps_array) - num_samples_tokeep
                if number_of_samples_to_drop > 0:
                    self.time_stamps_array = self.time_stamps_array[number_of_samples_to_drop:]

        # Update the FPS of the solvers
        if prev_fps != self.img_avrg_fps:
            if self.curr_intersection_type is IntersectionType.StopSign:
                self.ss_solver.update_fps(self.img_avrg_fps)
            elif self.curr_intersection_type is IntersectionType.TrafficLight:
                self.tl_solver.update_fps(self.img_avrg_fps)


    # CALLBACK SECTION
    def intersection_type_callback(self, msgs):
        """
        This method changes the internal intersection type and initiates / terminates the different values needed

        :param data: intersection data given by ROS
        """

        print("BaseComNode: intersection_type_callback")

        # Update only when we have information
        if len(msgs.infos) != 0:
            # new_intersection_type = IntersectionType(data.infos)
            tag_info = TagInfo()
            new_intersection_type = IntersectionType.Unknown

            # Loop over all detected info
            for info in msgs.infos:
                print(f"BaseComNode: intersection_type_callback: info.traffic_sign_type: {info.traffic_sign_type}")
                if (info.tag_type == tag_info.SIGN):
                    if (info.traffic_sign_type == tag_info.STOP):
                        new_intersection_type = IntersectionType.StopSign
                        break
                    elif (info.traffic_sign_type == tag_info.T_LIGHT_AHEAD):
                        new_intersection_type = IntersectionType.TrafficLight
                        break

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
        # print(self.curr_intersection_type)
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

            # Just before resetting we turn blue light for a moment
            self.blink_at(0, color="blue")
            sleep(2)
            self.ss_solver.reset()
            self.blink_at(self.ss_solver.blink_freq)

        print(f'points {len(self.ss_solver.point_buffer.points)}')
        buffer = self.ss_solver.point_buffer.points.copy()
        #for i, point in enumerate(buffer):
        #    #freq, spikes = point.get_frequency()
        #    freq, spikes = point.get_frequency(self.img_avrg_fps)
        #    print(f"{i} -- {freq}: {point}\n{i} -- {spikes}")
        return ActionState.Solving

    # OVERWRITTEN SECTION
    def blink_at(self, frequency: int, color: str = 'white'):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param frequency: frequency of blinking
        """
        print(f' blink_at self-freq: {frequency}')
        pass  # placeholder for inheritance

    # OVERWRITTEN SECTION
    def publish_signal(self, action: ActionState):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param action: the go signal
        """
        print(f'Action:{action}')
        pass  # placeholder for inheritance

    # INTERNAL SECTION
    def update_action_state(self, action_state):
        """
        All state updates should be done here
        """
        if action_state in [ActionState.Go, ActionState.TimedOut]:
            print(" update action state GO | Timeout")
            # Set the intersection to unknown so we stop processing
            self.begin_solving_time_sec = time()
            self.last_state_transition_time = time()

            # Publish signals and handle LED colors
            self.publish_signal(action_state)
