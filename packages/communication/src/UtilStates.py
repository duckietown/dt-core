from enum import Enum


# TODO perhaps merge some states?

# State relative to the intersection types
# TODO I think we should rename this one "IntersectionType" or State
class ComState(Enum):
    Sleeping = 0 # TODO Is this really needed? Perhaps use a more generic name "Unknown"?
    StopSign = 1
    TrafficLight = 2


# Actions states to handle overall behaviour
class ActionState(Enum):
    Go = 0         # State where we finally publish the GO signal and trun off LEDs
    SignalToGo = 1 # State just before the final GO, this is needed to give
                   # a visual cue (Can be solid Green for example)
    Solving = 2    # State to continue the solving the intersection priority/TL
    TimedOut = 3   # When we take too much time and no GO was given (duration TODO)


# States for TL processing
class TrafficLightState(Enum):
    SensingTL = 0   # State where TL is still being decoded
    RedTL = 1       # State where "Red" (solid) light is read
    GreenTL = 2     # State "green" (flashing) light is read


# States to be used for Stop Sign intersection nogiciation
class StopSignState(Enum):
    UnknownPriority = 0     # State where no priority has been established (still deconding LED from others)
    LowPriority = 1         # State where Other bot(s) has/have higher priority 
    HasHighestPriority = 2  # State where we have highest priority
