from statemachine import StateChart, State
from wr_statemachine.statemachine_enum import StateMachineEnum


class GNSSMission(StateChart):
    "The state machine for the GNSS Mission"

    def __init__(self, smpublisher, smsubscriber, *args, **kwargs):
        self.smpublisher = smpublisher
        self.smsubscriber = smsubscriber
        super().__init__(*args, **kwargs)

    default = State(initial=True)
    traversal = State()
    approaching = State()
    blink = State()

    path = (
        default.to(traversal, cond="is_active")
        | traversal.to(default, cond="mission_complete")
        | approaching.to(default, cond="mission_complete")
        | blink.to(default, cond="mission_complete")
        | traversal.to(approaching, cond="aruco_is_not_none")
        | approaching.to(blink, cond="reached_aruco")
        | blink.to(default)
    )

    "Conditional statements for state transition"

    def aruco_is_not_none(self) -> bool:
        """Use detection to find the aruco tag or be able to tell us if it's there and where"""
        return True  # A function that tells us whether or not the aruco tag is not none

    def reached_aruco(self) -> bool:
        """Uses distance"""
        return True

    def is_active(self):
        if self.smsubscriber.active_mission == StateMachineEnum.GNSS:
            return True
        return False

    "Actions to be carried out during the state"

    def on_enter_traversal(self):
        """Subscribe to /aruco to check if we can see the aruco tag
        and also use something in path planning to traverse there."""
        return None

    def on_enter_approaching(self):
        return None

    def on_enter_blink(self):
        return None

    "Mission complete condition"

    def mission_complete(self):
        if self.smsubscriber.active_mission != StateMachineEnum.GNSS:
            return True
        return False
