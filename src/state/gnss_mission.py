from statemachine import StateChart, State

class GNSSMission(StateChart):
    "The state machine for the GNSS Mission"
    default = State(initial=True)
    traversal = State()
    following = State()
    blink = State()

    path = (
        default.to(traversal) 
        | traversal.to(following)
        | following.to(blink)
        | blink.to(default)
    )