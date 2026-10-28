import rclpy

from astronaut_mission import AstronautMission
from person_monitor import PersonMonitor


def main():
    rclpy.init()
    monitor = PersonMonitor()
    mission = AstronautMission(monitor)

    # Re-check the transition conditions 10 times per second. Each tick tries
    # the "path" transitions from the current state and takes the first one
    # whose condition is true.
    monitor.create_timer(0.1, mission.path)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()