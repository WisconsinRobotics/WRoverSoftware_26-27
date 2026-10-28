# =============================================================================
# Background:
#   The rover uses GNSS measurements for geographic position,
#   wheel odometry for velocity, and IMU yaw for heading.
#
#   A Kalman filter estimates east/north position, velocity, yaw,
#   and angular velocity.
#   Pose publication stops when GNSS, odometry, or IMU updates become too old.
# =============================================================================
# Brief:
#   Combines GNSS, odometry, and IMU yaw to publish the rover's geographic pose.
# =============================================================================
# Subscribes to:
#   '/imu' (std_msgs/Float32)
#       Raw IMU yaw in degrees.
#   '/swerve' (std_msgs/Float32MultiArray)
#       [forward, strafe, left_trigger, right_trigger].
#   '/odometry' (geometry_msgs/Twist)
#       Description:
#           Wheel-derived body-frame velocity and omega.
#       Format:
#           linear:
#            x: forward (m/s)
#            y: strafe (m/s)
#            z: unused
#           angular.z: yaw rate in rad/s, CCW positive
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
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose2D, Twist

from ublox_ubx_msgs.msg import UBXNavPVT, GpsFix

from wr_localization.filter import RoverKalmanFilter
from wr_nav.navigation import has_timed_out, normalize_angle


UBX_DEG_SCALE = 1e-7

KALMAN_TIMESTEP = 0.05  # seconds, equivalent to 20 Hz

GOOD_FIX_TYPES = [
    GpsFix.GPS_FIX_2D,
    GpsFix.GPS_FIX_3D,
    GpsFix.GPS_PLUS_DEAD_RECKONING,
]
GOOD_FIX_TIMEOUT = 5.0
GOOD_ODOM_TIMEOUT = 2.0
GOOD_IMU_TIMEOUT = 2.0

# TODO: calibrate these
SWERVE_MPS = 3.0  # m/s
SWERVE_RADPS = 1.0  # rad/s


class Localization(Node):
    def __init__(self):
        super().__init__("localization")

        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)

        self.filter = RoverKalmanFilter()
        self.yaw = (
            None  # note: this is NOT the final yaw!! this is JUST the yaw from the imu.
        )
        self.last_good_fix = None
        self.last_odometry = None
        self.last_imu = None
        self.last_prediction = self.get_clock().now()
        self.swerve_control = [0.0, 0.0, 0.0]

        self.yaw_offset = math.radians(self.declare_parameter("yaw_offset", 0.0).value)

        # ---------------------------------------------------------------------
        # IMU Subscriber
        # ---------------------------------------------------------------------

        self.imu_subscription_ = self.create_subscription(
            Float32, "/imu", self.imu_callback, 10
        )

        # ---------------------------------------------------------------------
        # Odometry Subscriber
        # ---------------------------------------------------------------------

        self.odometry_subscription_ = self.create_subscription(
            Twist, "/odometry", self.odometry_callback, 10
        )

        # ---------------------------------------------------------------------
        # GNSS Subscriber
        # ---------------------------------------------------------------------

        self.gnss_subscription_ = self.create_subscription(
            UBXNavPVT, "/ubx_nav_pvt", self.gnss_callback, 10
        )

        self.swerve_subscription_ = self.create_subscription(
            Float32MultiArray, "/swerve", self.swerve_callback, 10
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
        self.predict(self.get_clock().now())
        self.publish_pose()

    def swerve_callback(self, msg: Float32MultiArray):
        forward, strafe, left_trigger, right_trigger = msg.data
        # TODO: if the max speed is SWERVE_MPS, then this assumes that the max speed is sqrt(2)*SWERVE_MPS
        # but if the max speed is SWERVE_MPS/sqrt(2), then the name is misleading AND max speed is dependent on direction?
        # This requires some form of normalization in control.
        self.swerve_control = (
            forward * SWERVE_MPS,
            strafe * SWERVE_MPS,
            (right_trigger - left_trigger) / 2.0 * SWERVE_RADPS,
        )

    def imu_callback(self, msg: Float32):
        now = self.get_clock().now()
        if not math.isfinite(msg.data):
            return
        self.yaw = normalize_angle(math.radians(msg.data) + self.yaw_offset)
        self.filter.correct_imu(self.yaw)
        self.last_imu = now
        self.publish_pose()

    def odometry_callback(self, msg: Twist):
        now = self.get_clock().now()
        # TODO: should we keep track of odom even when the imu times out?
        if self.yaw is None or has_timed_out(self.last_imu, GOOD_IMU_TIMEOUT, now):
            return
        fwd, left = msg.linear.x, msg.linear.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        # TODO: use an EKF to model yaw uncertainty
        # or just add extra noise
        self.filter.correct_odometry(
            fwd * c - left * s, fwd * s + left * c, float(msg.angular.z)
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
        self.filter.correct_gnss(
            msg.lat * UBX_DEG_SCALE,
            msg.lon * UBX_DEG_SCALE,
            msg.h_acc * 1e-3,  # UBX horizontal accuracy is in millimeters.
        )
        self.last_good_fix = now
        self.publish_pose()

    def predict(self, now: Time):
        """Advance the filter only on the 20 Hz prediction timer."""
        previous = self.last_prediction
        self.last_prediction = now
        if previous is None:
            return

        dt = (now - previous).nanoseconds * 1e-9
        # TODO: handle funky timer business.
        if not has_timed_out(self.last_imu, GOOD_IMU_TIMEOUT, now):
            forward, left, omega = self.swerve_control
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            # TODO: use an EKF to account for heading uncertainty, as above.
            u = np.array([forward * c - left * s, forward * s + left * c, omega])
            self.filter.predict(dt, u)
        else:
            self.filter.predict(dt)

    def publish_pose(self):
        """Publish what the current Kalman filter thinks our pose is."""
        now = self.get_clock().now()
        if (
            has_timed_out(self.last_good_fix, GOOD_FIX_TIMEOUT, now)
            or has_timed_out(self.last_odometry, GOOD_ODOM_TIMEOUT, now)
            or has_timed_out(self.last_imu, GOOD_IMU_TIMEOUT, now)
        ):
            return
        self.pose = self.filter.pose()
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
