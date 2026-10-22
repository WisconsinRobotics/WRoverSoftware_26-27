from statemachine import StateChart, State

class AstronautMission(StateChart):
    "The statemachine for the astronaut mission"
    default = State(initial=True)
    following = State()
    tracking = State()
    finding = State()

    path = (
        default.to(finding) 
        | finding.to(tracking, cond=is_person_found) 
        | tracking.to(following, cond=follow_gesture)
        | following.to(tracking, cond=stay_gesture)
        | following.to(finding, cond=is_person_found)
        | tracking.to(finding, cond=is_person_lost)
    )

    def is_person_found() -> bool:
        return # function from detection to see if we found the astronaut

    def is_person_lost() -> bool:
        return # function from detection to see if we can still see the person

    def follow_gesture() -> bool:
        return # function from detection to see if the following gesture was made

    def stay_gesture() -> bool:
        return # function from detecttion to see if the stay gesture was made

