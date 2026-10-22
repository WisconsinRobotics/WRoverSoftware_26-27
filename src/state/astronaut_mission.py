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

    "Conditional statements for state transition"
    def is_person_found(self) -> bool:
        return True # function from detection to see if we found the astronaut

    def is_person_lost(self) -> bool:
        return True # function from detection to see if we can still see the person

    def follow_gesture(self) -> bool:
        return True # function from detection to see if the following gesture was made

    def stay_gesture(self) -> bool:
        return True # function from detecttion to see if the stay gesture was made

    "Actions to be carried out during the state"
    def on_following(self):
        return None 

    def on_tracking(self):
        return None

    def on_finding(self):
        return None

    # On default will be a more global command potentially declared in a main function