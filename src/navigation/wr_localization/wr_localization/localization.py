# =============================================================================
# Background:
#   The rover uses GNSS (Global Navigation Satellite System) measurements for
#   geographic position and odometry for yaw and velocity when available.
#
#   A Kalman filter estimates east/north position and velocity. GNSS corrects
#   position and odometry corrects velocity, each weighted by its uncertainty.
#   Pose publication stops when
#   either the last valid fix or the last odometry update becomes too old.
# =============================================================================
# Brief:
#   Combines GNSS and odometry to publish the rover's geographic pose.
# =============================================================================
# Subscribes to:
#   '/odometry' (wr_navigation_msgs/OdometryState)
#       Description:
#           Odometry yaw and body-frame velocity.
#       Format:
#           yaw (float64)
#           velocity (Point): x forward, y left, z zero, in m/s
#           omega: yaw rate in rad/s, CCW positive
#       Example:
#           3.14
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
from rclpy.time import Time
from wr_navigation_msgs.msg import OdometryState
from geometry_msgs.msg import Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT, GpsFix

from wr_localization.filter import PositionKalmanFilter


UBX_DEG_SCALE = 1e-7

KALMAN_TIMESTEP = 0.05  # seconds, equivalent to 20 Hz

GOOD_FIX_TYPES = [
    GpsFix.GPS_FIX_2D,
    GpsFix.GPS_FIX_3D,
    GpsFix.GPS_PLUS_DEAD_RECKONING,
]
GOOD_FIX_TIMEOUT = 5.0
GOOD_ODOM_TIMEOUT = 2.0


def normalize_angle(angle: float) -> float:
    """Binds the angle from -pi to pi, inclusive."""
    return math.atan2(math.sin(angle), math.cos(angle))


def has_timed_out(last_update: Time | None, timeout: float, now: Time) -> bool:
    """Check whether an update is missing or older than the timeout in seconds."""
    if last_update is None:
        return True
    since_last_update = (now - last_update).nanoseconds * 1e-9
    return since_last_update > timeout


class Localization(Node):
    def __init__(self):
        super().__init__("localization")

        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)

        self.pos_filter = PositionKalmanFilter()
        self.yaw = None
        self.last_good_fix = None
        self.last_odometry = None
        self.last_prediction = None

        # ---------------------------------------------------------------------
        # Odometry Subscriber
        # ---------------------------------------------------------------------

        self.odometry_subscription_ = self.create_subscription(
            OdometryState, "/odometry", self.odometry_callback, 10
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

        self.prediction_timer_ = self.create_timer(
            KALMAN_TIMESTEP, self.prediction_callback
        )

        self.get_logger().info("Started localization node.")

    def prediction_callback(self):
        """Advance position and uncertainty even when sensors stop updating."""
        now = self.get_clock().now()
        self.predict(now)
        self.publish_pose()

    def odometry_callback(self, msg: OdometryState):
        now = self.get_clock().now()
        self.predict(now)
        self.yaw = normalize_angle(msg.yaw)
        forward, left = msg.velocity.x, msg.velocity.y
        self.pos_filter.correct_odometry(
            forward * math.cos(self.yaw) - left * math.sin(self.yaw),
            forward * math.sin(self.yaw) + left * math.cos(self.yaw),
        )
        self.last_odometry = now
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
        self.predict(now)
        self.pos_filter.correct_gnss(
            msg.lat * UBX_DEG_SCALE,
            msg.lon * UBX_DEG_SCALE,
            msg.h_acc * 1e-3,  # UBX horizontal accuracy is in millimeters.
        )
        self.last_good_fix = now
        self.publish_pose()

    def predict(self, now):
        """Advance the filter to this time before a timer tick or correction."""
        previous = self.last_prediction
        self.last_prediction = now
        if previous is None:
            return

        dt = (now - previous).nanoseconds * 1e-9
        # TODO: handle funky timer business.
        self.pos_filter.predict(dt)

    def publish_pose(self):
        """Publish what the current Kalman filter thinks our pose is."""
        now = self.get_clock().now()
        if has_timed_out(self.last_good_fix, GOOD_FIX_TIMEOUT, now) or has_timed_out(
            self.last_odometry, GOOD_ODOM_TIMEOUT, now
        ):
            return
        lat, lon = self.pos_filter.coordinates()
        self.pose = Pose2D(x=lat, y=lon, theta=self.yaw)
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
