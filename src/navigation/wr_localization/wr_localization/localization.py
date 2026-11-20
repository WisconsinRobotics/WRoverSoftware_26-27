# =============================================================================
# Background:
#   The rover uses the latest GNSS fix for geographic position
#   and the latest IMU yaw for heading.
#   Pose publication stops when GNSS or IMU updates become too old.
# =============================================================================
# Brief:
#   Publishes the rover's geographic pose from GNSS and IMU yaw.
# =============================================================================
# Subscribes to:
#   '/imu' (std_msgs/Float32)
#       Raw IMU yaw in degrees.
#   '/ubx_nav_pvt' (ublox_ubx_msgs/UBXNavPVT)
#       Description:
#           GNSS receiver data, including latitude and longitude data.
#       Format:
#           ...
#           lon: float
#           lat: float
#           ...
#       Example:
#           ...
#           lon: -894012345
#           lat: 433821234
#           ...
#
# =============================================================================
# Publishes to:
#   '/pose' (geometry_msgs/Pose2D)
#       Description:
#           Publishes our geographic coordinates and heading as a pose.
#       Format:
#           x: latitude (degrees)
#           y: longitude (degrees)
#           theta: heading (radians)
#       Example:
#           x: 43.07
#           y: -89.4
#           theta: 39.2
#       Notes:
#           Theta should be CCW positive.
# =============================================================================

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT, GpsFix

from wr_nav.navigation import has_timed_out, normalize_angle


UBX_DEG_SCALE = 1e-7

GOOD_FIX_TYPES = [
    GpsFix.GPS_FIX_2D,
    GpsFix.GPS_FIX_3D,
    GpsFix.GPS_PLUS_DEAD_RECKONING,
]
GOOD_FIX_TIMEOUT = 5.0
GOOD_IMU_TIMEOUT = 2.0


class Localization(Node):
    def __init__(self):
        super().__init__("localization")

        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)

        self.last_good_fix = None
        self.last_imu = None

        self.yaw_offset = math.radians(self.declare_parameter("yaw_offset", 0.0).value)

        # ---------------------------------------------------------------------
        # IMU Subscriber
        # ---------------------------------------------------------------------

        self.imu_subscription_ = self.create_subscription(
            Float32, "/imu", self.imu_callback, 10
        )

        # ---------------------------------------------------------------------
        # GNSS Subscriber
        # ---------------------------------------------------------------------

        self.gnss_subscription_ = self.create_subscription(
            UBXNavPVT, "/ubx_nav_pvt", self.gnss_callback, 10
        )

        # ---------------------------------------------------------------------
        # Pose Publisher
        # ---------------------------------------------------------------------

        self.pose_publisher_ = self.create_publisher(Pose2D, "/pose", 10)

        self.get_logger().info("Started localization node.")

    def imu_callback(self, msg: Float32):
        now = self.get_clock().now()
        if not math.isfinite(msg.data):
            return
        self.pose.theta = normalize_angle(math.radians(msg.data) + self.yaw_offset)
        self.last_imu = now
        self.publish_pose()

    def gnss_callback(self, msg: UBXNavPVT):
        now = self.get_clock().now()
        # Message might not have the information we want.
        if (
            not msg.gnss_fix_ok
            or msg.gps_fix.fix_type not in GOOD_FIX_TYPES
            or msg.invalid_llh
            or not math.isfinite(msg.lat)
            or not math.isfinite(msg.lon)
            or not math.isfinite(msg.h_acc)
            or msg.h_acc < 0
        ):
            return
        self.pose.x = msg.lat * UBX_DEG_SCALE
        self.pose.y = msg.lon * UBX_DEG_SCALE
        self.last_good_fix = now
        self.publish_pose()

    def publish_pose(self):
        """Publish the latest GNSS position and IMU heading."""
        now = self.get_clock().now()
        if has_timed_out(self.last_good_fix, GOOD_FIX_TIMEOUT, now) or has_timed_out(
            self.last_imu, GOOD_IMU_TIMEOUT, now
        ):
            return
        self.pose_publisher_.publish(self.pose)


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
