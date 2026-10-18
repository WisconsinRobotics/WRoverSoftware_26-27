# =============================================================================
# Background:
#   The rover uses GNSS (Global Navigation Satellite System) measurements for
#   geographic position and a CTRE Phoenix6 Pigeon2 IMU for yaw. GNSS provides
#   longitude and latitude, while the IMU tracks changes in orientation.
#
#   Only valid GNSS fixes update position. Pose publication stops when
#   either the last valid fix or the last IMU update becomes too old.
# =============================================================================
# Brief:
#   Combines GNSS position and IMU yaw to publish the rover's geographic pose.
# =============================================================================
# Subscribes to:
#   '/imu' (std_msgs/Float32)
#       Description:
#           Raw Pigeon2 yaw in degrees, published at 100 Hz.
#       Format:
#           yaw (float32)
#       Example:
#           180.0
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

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT, GpsFix


class Localization(Node):
    UBX_DEG_SCALE = 1e-7

    GOOD_FIX_TYPES = [
        GpsFix.GPS_FIX_2D,
        GpsFix.GPS_FIX_3D,
        GpsFix.GPS_PLUS_DEAD_RECKONING,
    ]
    GOOD_FIX_TIMEOUT = 5.0
    GOOD_IMU_TIMEOUT = 2.0

    def __init__(self):
        super().__init__("localization")

        # This is NOT the initial position of the robot, this is just the position used for imu correction.
        self.init_pos = None
        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)

        self.last_good_fix = None
        self.last_imu = None

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
        if not math.isfinite(msg.data):
            return
        self.pose.theta = math.radians(msg.data)
        self.last_imu = self.get_clock().now()
        self.publish_pose()

    def gnss_callback(self, msg: UBXNavPVT):
        # Message might not have the information we want.
        if (
            not msg.gnss_fix_ok
            or msg.gps_fix.fix_type not in self.GOOD_FIX_TYPES
            or msg.invalid_llh
            or not math.isfinite(msg.lat)
            or not math.isfinite(msg.lon)
        ):
            return
        self.last_good_fix = self.get_clock().now()
        self.pose.x = msg.lat * self.UBX_DEG_SCALE
        self.pose.y = msg.lon * self.UBX_DEG_SCALE
        self.publish_pose()

    def publish_pose(self):
        """Publish our current pose while both GNSS and IMU updates are fresh."""
        if None in (self.last_good_fix, self.last_imu):
            return
        now = self.get_clock().now()
        since_last_good_fix = (now - self.last_good_fix).nanoseconds * 1e-9
        since_last_imu = (now - self.last_imu).nanoseconds * 1e-9
        if (
            since_last_good_fix < self.GOOD_FIX_TIMEOUT
            and since_last_imu < self.GOOD_IMU_TIMEOUT
        ):
            self.pose_publisher_.publish(self.pose)

    def get_pos(self) -> Point:
        """Get position as a Point with x=latitude and y=longitude in degrees."""
        return Point(x=self.pose.x, y=self.pose.y)

    def normalize_angle(self, angle: float) -> float:
        """Binds the angle from -pi to pi."""
        return math.atan2(math.sin(angle), math.cos(angle))


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
