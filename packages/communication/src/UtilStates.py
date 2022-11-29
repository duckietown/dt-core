from enum import Enum


# TODO perhaps merge some states?

# Intersection types
class IntersectionType(Enum):
    Unknown = 0  # TODO Is this really needed? Perhaps use a more generic name "Unknown"?
    StopSign = 1
    TrafficLight = 2


# Actions states to handle overall behavior
class ActionState(Enum):
    Solving = 0   # State when still solving the intersection priority/TL
    Go = 1        # State where we finally publish the GO signal and turn off LEDs
    TimedOut = 2  # When we take too much time and no GO was given (duration TODO)


# States for TL processing
class TrafficLightState(Enum):
    Sensing = 0   # State where TL is still being decoded
    Green = 1     # State "green" (flashing) light is read
    #Red = 2       # State where "Red" (solid) light is read TODO probably not needed
