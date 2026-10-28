from statemachine import StateChart, State

class GNSSMission(StateChart):
    "The state machine for the GNSS Mission"
    default = State(initial=True)
    traversal = State()
    following = State()
    blink = State()

    path = (
        default.to(traversal) 
        | traversal.to(following, cond="aruco_is_not_none")
        | following.to(blink, cond="reached_aruco")
        | blink.to(default)
    )

    "Conditional statements for state transition"
    def aruco_is_not_none(self) -> bool:
        return True # A function that tells us whether or not the aruco tag is not none

    def reached_aruco(self) -> bool:
        return True # A function that tells us that we are at the aruco tag


    "Actions to be carried out during the state"
    def on_traversal(self):
        return None

    def on_following(self):
        return None

    def on_blink(self):
        return None