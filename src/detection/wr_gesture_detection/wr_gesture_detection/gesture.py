"""
Code representation of the observed gesture.
"""

from enum import Enum


class Gesture(Enum):
    NONE = 1
    FOLLOW = 2
    STAY = 3
    FETCH = 4
    COME = 5
    GIVE = 6
