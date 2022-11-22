import numpy as np
from UtilStates import ComState, ActionState
from random import choice
from PointBuffer import PointBuffer
import cv2


class BaseComNode:
    """
    Base class for the communication node.
    This class is used to define behaviors for the node that do not directly interact with ROS. This makes testing
    easier
    """
    # legal frequencies that can be used for flashing
    LEGAL_FREQ = [2, 4, 5, 6, 8, 10, 15]

    def __init__(self, buffer_length=60, buffer_forget_time=40):
        # buffer used to keep active points
        self.buffer = PointBuffer(buffer_length, buffer_forget_time)
        # current state of the node. By default, it is inactive
        self.active_state = ComState.Sleeping

    def img_callback(self, data):
        """
        This method redirect the image data to the right callback function depending on the current node state

        :param data: raw image data given by ROS
        """
        if self.active_state is ComState.Sleeping:
            return
        elif self.active_state is ComState.StopSign:
            self.stop_sign_callback(data)
        elif self.active_state is ComState.TrafficLight:
            self.traffic_light_callback(data)

    def state_callback(self, data):
        """
        This method changes the internal state of the node and initiates / terminates the different values needed

        :param data: state data given by ROS
        """
        new_state = ComState(data)

        # TODO: fill the state machine for transitions
        if new_state is ComState.StopSign:
            self.blink_at(choice(self.LEGAL_FREQ))

        self.active_state = new_state

    def blink_at(self, frequency: int):
        """
        This method changes the blinking frequency of the DuckieBot to the one specified in the input

        :param frequency: frequency of blinking
        """
        pass  # placeholder for inheritance

    def stop_sign_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the state
        is in ``ComState.StopSign``. It then adds this image to the buffer

        :param data: raw image data given by ROS
        """
        new_img = cv2.cvtColor(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        self.buffer.push_frame(new_img)

    def traffic_light_callback(self, data):
        """
        This method is used as a callback when a new image is received by the node and the state
        is in ``ComState.TrafficLight``.

        :param data: raw image data given by ROS
        """
        # TODO: fill
        pass

    def run(self) -> ActionState:
        """
        Method that surveys the real world state and determines what to do

        :return: the go or wait signal
        """

        # TODO: fill
        for i, point in enumerate(self.buffer.points):
            print(f"{i} -- freq: {point.get_frequency()[0]} -- {point}")
        print()
        pass
