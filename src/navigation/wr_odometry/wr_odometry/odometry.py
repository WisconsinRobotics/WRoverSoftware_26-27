# =============================================================================
# TODO other documentation
# Subscribes to:
#   '/can_receive_swerve' (wr_control_msgs/CanReceiveSwerve)
#       Steering angles in degrees.
#   '/can_receive_drive' (wr_control_msgs/CanReceiveDrive)
#       Drive motor electrical RPMs.
# Publishes to:
#   '/odometry' (geometry_msgs/Twist)
#       linear:
#        x: forward (m/s)
#        y: strafe (m/s)
#       angular.z: yaw rate in rad/s, CCW positive
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wr_control_msgs.msg import CanReceiveDrive, CanReceiveSwerve
from wr_nav.navigation import has_timed_out, wrap_degrees

SENSOR_TIMEOUT = 1.0
# TODO: should be tau * radius / (60 * pole_pairs * motor_turns_per_wheel_turn)
# or something like that...
ERPM_TO_MPS = 1.0
# TODO: confirm wheel-center spacing in meters from wr_drive
WHEELBASE = 0.93
TRACK_WIDTH = 0.60

WHEEL_LOCATIONS = (
    (WHEELBASE / 2, TRACK_WIDTH / 2),  # FL
    (WHEELBASE / 2, -TRACK_WIDTH / 2),  # FR
    (-WHEELBASE / 2, TRACK_WIDTH / 2),  # BL
    (-WHEELBASE / 2, -TRACK_WIDTH / 2),  # BR
)


def encoder_to_wheel(deg: float) -> float:
    """This should invert the encoder reading to a proper swerve wheel angle."""
    return math.radians(wrap_degrees(deg - 180) * 4)


class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        self.twist = Twist()
        self.swerve_degs = None
        self.drive_rpms = None

        self.last_swerve = None
        self.last_drive = None

        self.swerve_subscription_ = self.create_subscription(
            CanReceiveSwerve, "/can_receive_swerve", self.swerve_callback, 10
        )
        self.drive_subscription_ = self.create_subscription(
            CanReceiveDrive, "/can_receive_drive", self.drive_callback, 10
        )
        self.odometry_publisher_ = self.create_publisher(Twist, "/odometry", 10)

        self.get_logger().info("Started odometry node.")

    def swerve_callback(self, msg: CanReceiveSwerve):
        self.swerve_degs = msg
        self.last_swerve = self.get_clock().now()
        self.publish_odometry()

    def drive_callback(self, msg: CanReceiveDrive):
        self.drive_rpms = msg
        self.last_drive = self.get_clock().now()
        self.publish_odometry()

    def update_state(self):
        """Adjust body-frame velocity and yaw rate from the eight wheel readings."""
        self.twist = Twist()
        rpms = (
            self.drive_rpms.fl_rpm,
            self.drive_rpms.fr_rpm,
            self.drive_rpms.bl_rpm,
            self.drive_rpms.br_rpm,
        )
        encoders = (
            self.swerve_degs.fl_encoder,
            self.swerve_degs.fr_encoder,
            self.swerve_degs.bl_encoder,
            self.swerve_degs.br_encoder,
        )
        for rpm, encoder, (x, y) in zip(rpms, encoders, WHEEL_LOCATIONS):
            angle = encoder_to_wheel(encoder)
            speed = rpm * ERPM_TO_MPS
            fwd, left = speed * math.cos(angle), speed * math.sin(angle)
            self.twist.linear.x += fwd / 4.0
            self.twist.linear.y += left / 4.0
            self.twist.angular.z += x * left - y * fwd
        self.twist.angular.z /= WHEELBASE**2 + TRACK_WIDTH**2

    def publish_odometry(self):
        now = self.get_clock().now()
        if has_timed_out(self.last_drive, SENSOR_TIMEOUT, now) or has_timed_out(
            self.last_swerve, SENSOR_TIMEOUT, now
        ):
            return
        self.update_state()
        self.odometry_publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
