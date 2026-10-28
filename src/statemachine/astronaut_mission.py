import math

from statemachine import StateChart, State
from wr_detection_msgs.msg import Person

# Person.position is in millimeters unless the detection node converts it
# (point_cloud_utils works in mm). Set this to 1.0 if /person is in meters.
POSITION_TO_M = 0.001

# Both are guesses to tune on the real rover. Don't make the stopping distance
# too small: up close the depth camera and pose estimation start to lose the
# person, which would drop you into "finding" instead of a clean stop.
ARRIVE_DISTANCE_M = 1.5  # COME stops this far from the astronaut
COME_RETRIGGER_M = 2.0  # COME only starts a new approach beyond this distance


class AstronautMission(StateChart):
    "The statemachine for the astronaut mission"

    def __init__(self, monitor, *args, **kwargs):
        # monitor: anything with a .person property (see person_monitor.py)
        self.monitor = monitor
        # FOLLOW keeps following until told otherwise; COME stops on arrival.
        self.continuous = False
        super().__init__(*args, **kwargs)

    default = State(initial=True)
    following = State()
    tracking = State()
    finding = State()

    # Conditions are given as strings because the methods are defined below.
    # Transitions from the same state are tried in the order listed.
    path = (
        default.to(finding)
        | finding.to(tracking, cond="is_person_found")
        | tracking.to(following, cond="follow_gesture", on="start_follow")
        | tracking.to(following, cond="come_gesture", on="start_come")
        | following.to(tracking, cond="stay_gesture")
        | following.to(tracking, cond="arrived")
        | following.to(finding, cond="is_person_lost")
        | tracking.to(finding, cond="is_person_lost")
    )

    "Helpers"

    def _distance_m(self):
        """Horizontal distance to the person in meters, or None if unknown."""
        p = self.monitor.person
        if p is None or p.position.z <= 0:  # z <= 0 means no valid depth
            return None
        return math.hypot(p.position.x, p.position.z) * POSITION_TO_M

    "Conditional statements for state transition"

    def is_person_found(self) -> bool:
        return self.monitor.person is not None

    def is_person_lost(self) -> bool:
        return self.monitor.person is None

    def follow_gesture(self) -> bool:
        p = self.monitor.person
        return p is not None and p.gesture == Person.FOLLOW

    def come_gesture(self) -> bool:
        p = self.monitor.person
        d = self._distance_m()
        return (
            p is not None
            and p.gesture == Person.COME
            and d is not None
            and d > COME_RETRIGGER_M
        )

    def stay_gesture(self) -> bool:
        p = self.monitor.person
        return p is not None and p.gesture == Person.STAY

    def arrived(self) -> bool:
        # Only COME ends on its own; FOLLOW runs until STAY or the person is lost.
        d = self._distance_m()
        return (not self.continuous) and d is not None and d < ARRIVE_DISTANCE_M

    "Remember which gesture started the current following"

    def start_follow(self):
        self.continuous = True

    def start_come(self):
        self.continuous = False

    "Actions to be carried out during the state"

    def on_enter_following(self):
        return None

    def on_enter_tracking(self):
        return None

    def on_enter_finding(self):
        return None