# =============================================================================
# TODO
# =============================================================================
# Subscribes to:
#   '/localization' (geometry_msgs/Pose2D)
#       x: latitude (degrees)
#       y: longitude (degrees)
#       theta: ignored
#   '/target_gnss' (geometry_msgs/Point)
#       x: target latitude (degrees)
#       y: target longitude (degrees)
#       z: ignored
#   '/nav_mode' (std_msgs/Int32)
#       data: 4 enables distance publishing; other modes disable it
# =============================================================================
# Publishes to:
#   '/distance_to_target' (std_msgs/Float32)
#       data: horizontal distance to the target (meters)
# =============================================================================

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Float32, Int32


class Distance(Node):
    GNSS_TARGET_MODE = 4

    def __init__(self):
        super().__init__("distance")

        self.pose = None
        self.gnss_target = None
        self.nav_mode = 0

        self.localization_subscriber_ = self.create_subscription(
            Pose2D, "/localization", self.localization_callback, 10
        )
        self.target_subscriber_ = self.create_subscription(
            Point, "/target_gnss", self.target_callback, 10
        )
        self.nav_mode_subscriber_ = self.create_subscription(
            Int32, "/nav_mode", self.nav_mode_callback, 10
        )
        self.distance_publisher_ = self.create_publisher(
            Float32, "/distance_to_target", 10
        )

        self.get_logger().info("Started distance node.")

    def localization_callback(self, msg: Pose2D):
        self.pose = msg
        self.publish_distance()

    def target_callback(self, msg: Point):
        self.gnss_target = msg
        self.publish_distance()

    def nav_mode_callback(self, msg: Int32):
        self.nav_mode = msg.data
        self.publish_distance()

    def publish_distance(self):
        if (
            self.nav_mode != self.GNSS_TARGET_MODE
            or self.pose is None
            or self.gnss_target is None
        ):
            return
        dpos = self.geo_to_dpos(Point(x=self.pose.x, y=self.pose.y), self.gnss_target)
        self.distance_publisher_.publish(Float32(data=math.hypot(dpos.x, dpos.y)))

    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        """
        Takes two different geographic coordinates and spits out a difference in meters.
        There is an error of approximately 1-3m depending on direction after 1km of travel.
        """
        EARTH_RADIUS = 6371000.0

        dlat = math.radians(end.x - start.x)
        dlon = math.radians(end.y - start.y)

        mid_lat = math.radians((start.x + end.x) / 2)
        return Point(x=dlon * EARTH_RADIUS * math.cos(mid_lat), y=dlat * EARTH_RADIUS)


# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = Distance()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
