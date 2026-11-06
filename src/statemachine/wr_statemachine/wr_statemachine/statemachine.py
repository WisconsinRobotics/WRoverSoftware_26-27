import rclpy

from rclpy.executors import SingleThreadedExecutor

# from astronaut_mission import AstronautMission
# from gnss_mission import GNSSMission
from nav_publisher import NavPublisher
from person_monitor import PersonMonitor


def main():
    rclpy.init()
    monitor = PersonMonitor()
    monitor.create_timer(0.1, monitor)
    nav_publisher = NavPublisher()
    nav_publisher.create_timer(0.1, nav_publisher)

    # astronaut_mission = AstronautMission(nav_publisher, monitor)
    # gnss_mission = GNSSMission(nav_publisher)

    executor = SingleThreadedExecutor()
    executor.add_node(nav_publisher)
    executor.add_node(monitor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()  # not sure if I should do this
        monitor.destroy_node()
        nav_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
