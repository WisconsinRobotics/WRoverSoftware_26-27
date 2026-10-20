"""
Docstring
"""

__author__ = "Cameron Myhre"

from dataclasses import dataclass
from geometry_msgs.msg import Point
from .gesture import Gesture


@dataclass
class Person:
    """
    A dataclass representing a person detected by the gesture detection
    pipeline.

    Attributes:
        position:
            The estimated geometric median of the detected person, as a ROS2 Point.

        gesture:
            The observed gesture the person is preforming. This is an enum value
            with gestures corresponding as follows:

                1. None
                2. Follow
                3. Stay
                4. Fetch
                5. Come
                6. Give
    """

    def __init__(self, position: Point, gesture: Gesture, confidence: float):
        self._position = position
        self._gesture = gesture
        self._confidence = confidence

    @property
    def position(self):
        return self._position

    @property
    def gesture(self):
        return self._gesture

    @property
    def confidence(self):
        return self._confidence
