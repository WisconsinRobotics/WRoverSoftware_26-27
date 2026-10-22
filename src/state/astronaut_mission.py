from statemachine import StateChart, State

class AstronautMission(StateChart):
    "The statemachine for the astronaut mission"
    default = State(initial=True)
    following = State()
    tracking = State()
    finding = State()

    path = (
        default.to(finding) 
        | finding.to(tracking) 
        | tracking.to(following)
        | following.to(tracking)
        | following.to(finding)
        | tracking.to(finding)
    )

