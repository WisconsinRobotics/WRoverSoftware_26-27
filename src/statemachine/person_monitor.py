"""Remembers the latest /person message so the state machine can ask about it."""

import time

from rclpy.node import Node
from wr_detection_msgs.msg import Person


class PersonMonitor(Node):
    """Subscribes to /person and keeps only the most recent message.

    `person` is None when nothing has arrived for `timeout_s`. The detection
    node is expected to stop publishing when it can't see anyone, so "no recent
    message" is what "person lost" means here. Person.msg has no timestamp, so
    freshness is measured from when the message arrived.
    """

    def __init__(self, timeout_s: float = 1.0):
        super().__init__("person_monitor")
        self._timeout_s = timeout_s
        self._msg = None
        self._received_at = 0.0
        # If nothing arrives, check `ros2 topic info /person -v` for a QoS
        # mismatch (best-effort publisher) and use qos_profile_sensor_data.
        self.create_subscription(Person, "/person", self._on_person, 10)

    def _on_person(self, msg: Person):
        self._msg = msg
        self._received_at = time.monotonic()

    @property
    def person(self):
        """Latest Person message, or None if there isn't a recent one."""
        if self._msg is None or time.monotonic() - self._received_at > self._timeout_s:
            return None
        return self._msg