from enum import Enum


class ComState(Enum):
    Sleeping = 0
    StopSign = 1
    TrafficLight = 2


class ActionState(Enum):
    Go = 0
    Wait = 1
