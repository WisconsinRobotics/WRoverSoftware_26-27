This is the folder for the state machine of the rover with all it's tasks and transitions between tasks for the autonomous mission

Reloading the statemachine diagrams:
python3 -m statemachine.contrib.diagram gnss_mission.GNSSMission gnss_mission.png
python3 -m statemachine.contrib.diagram astronaut_mission.AstronautMission astronaut_mission.png

python-statemachine docs:
https://python-statemachine.readthedocs.io/en/latest/
