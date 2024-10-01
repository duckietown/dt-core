from dataclasses import dataclass

import numpy as np


@dataclass
class ThreeDimVec(object):
    """ Struct to store three variables referenced as x,y,z """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __str__(self):
        return "x: %f, y: %f, z: %f" % (self.x, self.y, self.z)

    def __mul__(self, other):
        return ThreeDimVec(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __div__(self, other):
        return ThreeDimVec(self.x / other, self.y / other, self.z / other)

    def __add__(self, other):
        return ThreeDimVec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return ThreeDimVec(self.x - other.x, self.y - other.y, self.z - other.z)

    def magnitude(self):
        return np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def planar_magnitude(self):
        return np.sqrt(self.x * self.x + self.y * self.y)


class Position(ThreeDimVec):
    """ Struct to store position components x,y,z"""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Position, self).__init__(x, y, z)


class Velocity(ThreeDimVec):
    """ Struct to store velocity components x,y,z"""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Velocity, self).__init__(x, y, z)


class Error(ThreeDimVec):
    """ Struct to store 3D errors which is in the form x,y,z"""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Error, self).__init__(x, y, z)


class RPY(ThreeDimVec):
    """ Struct to store the roll, pitch, in x,y,z"""

    def __init__(self, r=0.0, p=0.0, y=0.0):
        super(RPY, self).__init__(r, p, y)
        self.r = self.x
        self.p = self.y
        self.y = self.z

    def get_rpy(self):
        # the base class functions are not defined for rpy
        self.r = self.x
        self.p = self.y
        self.y = self.z
        return self.r, self.p, self.y
