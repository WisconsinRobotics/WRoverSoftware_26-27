"""Remembers the latest /person message so the state machine can ask about it."""

import time

from rclpy.node import Node
from wr_detection_msgs.msg import Person
from std_msgs.msg import Int32
from wr_statemachine.statemachine_enum import StateMachineEnum


# class PersonMonitor(Node):
#     """Subscribes to /person and keeps only the most recent message.

#     `person` is None when nothing has arrived for `timeout_s`. The detection
#     node is expected to stop publishing when it can't see anyone, so "no recent
#     message" is what "person lost" means here. Person.msg has no timestamp, so
#     freshness is measured from when the message arrived.
#     """

#     def __init__(self, timeout_s: float = 1.0):
#         super().__init__("person_monitor")
#         self._timeout_s = timeout_s
#         self._msg = None
#         self._received_at = 0.0
#         # If nothing arrives, check `ros2 topic info /person -v` for a QoS
#         # mismatch (best-effort publisher) and use qos_profile_sensor_data.
#         self.create_subscription(Person, "/person", self._on_person, 10)

#     def _on_person(self, msg: Person):
#         self._msg = msg
#         self._received_at = time.monotonic()

#     @property
#     def person(self):
#         """Latest Person message, or None if there isn't a recent one."""
#         if self._msg is None or time.monotonic() - self._received_at > self._timeout_s:
#             return None
#         return self._msg

# """A node used to publish to /nav_mode"""
# class NavPublisher(Node):
#     def __init__(self):
#         super().__init__("nav_publisher")
#         self.publisher = self.create_publisher(Int32, "/nav_mode", 1)

# """A node that subscribes to /mission_type"""
# class MissionType(Node):
#     def __init__(self):
#         super().__init__("mission_type")
#         self.subscription = self.create_subscription(Int32, "/mission_type", self.do_something_about_the_mission_change)


#     def do_something_about_the_mission_change(mission_type : Int32):
#         return None
# ------------------------------------------------------------------------------------------------
class SMPublisher(Node):
    def __init__(self):
        super().__init__("sm_publisher")
        self.nav_publisher = self.create_publisher(Int32, "/nav_mode", 1)


class SMSubscriber(Node):
    def __init__(self, timeout_s: float = 1.0):
        super().__init__("sm_subscriber")
        self.mission_type = self.create_subscription(
            Int32, "/mission_type", self.do_something_about_the_mission_change, 1
        )
        self.person_monitor = self.create_subscription(
            Person, "/person", self._on_person, 10
        )
        # self.aruco = self.create_subscription(, "/aruco", 1)
        # self.distance = self.create_subscription(Float32, "distance_to_target", qos_profile=1)

        self._timeout_s = (
            timeout_s  # somehow isolate these variables to only person monitor
        )
        self._msg = None
        self._received_at = 0.0

        self.active_mission = StateMachineEnum.NOMISSION

    def _on_person(self, msg: Person):
        self._msg = msg
        self._received_at = time.monotonic()

    @property
    def person(self):
        """Latest Person message, or None if there isn't a recent one."""
        if self._msg is None or time.monotonic() - self._received_at > self._timeout_s:
            return None
        return self._msg

    def do_something_about_the_mission_change(self, mission_type: Int32):
        # check mission type, change if it's different from previous mission type
        # if self.active_mission != mission_type.val:
        #      if self.active_mission == StateMachineEnum.NOMISSION:
        #         if self.active_mission ==
        #      ...

        "Really all I need to do"
        self.active_mission = mission_type.data
        return None
