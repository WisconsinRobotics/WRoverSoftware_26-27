# =============================================================================
# Background:
# TODO: all of this
# =============================================================================
# Brief:
#
# =============================================================================
# Subscribes to:
#   /imu
#   /ubx_nav_pvt
#
# =============================================================================
# Publishes to:
#
# =============================================================================

import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT


class Localization(Node):
    def __init__(self):
        super().__init__("localization")

        self.init_pos = None
        self.pose = Pose2D()
        self.true_north = 0.0

        # ---------------------------------------------------------------------
        # IMU Subscriber
        # ---------------------------------------------------------------------

        self.imu_subscription_ = self.create_subscription(Float32, "/imu", self.imu_callback, 10)

        # ---------------------------------------------------------------------
        # GNSS Subscriber
        # ---------------------------------------------------------------------

        self.gnss_subscription_ = self.create_subscription(UBXNavPVT, "/ubx_nav_pvt", self.gnss_callback, 10)

        # ---------------------------------------------------------------------
        # Pose Publisher
        # ---------------------------------------------------------------------

        self.pose_publisher_ = self.create_publisher(Pose2D, "/pose", 10)

    def imu_callback(self, msg: Float32):
        if self.true_north is not None:
            # TODO: Normalize this angle, and make sure that this heading has the correct sign.
            self.pose.heading = (msg.data - self.true_north)
        # TODO: Figure out if pose should publish without heading data.
        # This must be accounted for by all of /pose's subscribers.
        if self.pose.x is not None and self.pose.y is not None and self.pose.heading is not None:
            self.pose_publisher_.publish(self.pose)

    def gnss_callback(self, msg: UBXNavPVT):
        # This could be called from some explicit initialization event.
        if self.init_pos is None:
            self.init_pos = Point(msg.lat, msg.lon)
        # TODO: When confident enough, use the inverse tangent from these two positions to find heading correction.
        # This condition is just a placeholder.
        if math.hypot(msg.lat - self.init_pos.x, msg.lon - self.init_pos.y) < 0.2 * 1e7:
            # TODO: Use correct coordinates to calculate true north.
            self.true_north = math.atan2(
                math.radians(msg.lon - self.init_pos.y),
                math.radians(msg.lat - self.init_pos.x)
            )
        if self.pose.x is not None and self.pose.y is not None and self.pose.heading is not None:
            self.pose_publisher_.publish(self.pose)

    # TODO: Takes two different geographic coordinates and spits out a difference in meters.
    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        pass

# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = Localization()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
