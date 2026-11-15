import rclpy

from rclpy.executors import SingleThreadedExecutor

from wr_statemachine.astronaut_mission import AstronautMission
from wr_statemachine.gnss_mission import GNSSMission
from wr_statemachine.nodes import SMPublisher, SMSubscriber


def main():
    rclpy.init()
    smsubscriber = SMSubscriber()
    smpublisher = SMPublisher()

    missions = [
        AstronautMission(smpublisher, smsubscriber),
        GNSSMission(smpublisher, smsubscriber),
    ]

    def tick():
        for mission in missions:
            mission.send("path")

    smsubscriber.create_timer(0.1, tick)

    executor = SingleThreadedExecutor()
    executor.add_node(smsubscriber)
    executor.add_node(smpublisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()  # not sure if I should do this

        # Do I need the below ones?
        smpublisher.destroy_node()
        smsubscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
