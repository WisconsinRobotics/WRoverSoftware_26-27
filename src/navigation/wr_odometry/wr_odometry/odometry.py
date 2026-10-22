# =============================================================================
# TODO other documentation
# Subscribes to:
#   '/can_receive_swerve' (wr_control_msgs/CanReceiveSwerve)
#       Steering angles in degrees.
#   '/can_receive_drive' (wr_control_msgs/CanReceiveDrive)
#       Drive motor electrical RPMs.
# Publishes to:
#   '/odometry' (wr_navigation_msgs/OdometryState)
#       velocity (Point): x forward, y left, z zero.
#       omega: yaw rate in rad/s, CCW positive about the rover center.
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from wr_control_msgs.msg import CanReceiveDrive, CanReceiveSwerve
from wr_navigation_msgs.msg import OdometryState


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


def has_timed_out(last_update: Time | None, timeout: float, now: Time) -> bool:
    """Check if update has timed out or not."""
    if last_update is None:
        return True
    since_last_update = (now - last_update).nanoseconds * 1e-9
    return since_last_update > timeout


class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        self.odometry = OdometryState()
        self.raw_encoders = None
        self.drive_rpms = None

        self.last_encoders = None
        self.last_drive = None

        self.encoder_subscription_ = self.create_subscription(
            CanReceiveSwerve, "/can_receive_swerve", self.encoder_callback, 10
        )
        self.drive_subscription_ = self.create_subscription(
            CanReceiveDrive, "/can_receive_drive", self.drive_callback, 10
        )
        self.odometry_publisher_ = self.create_publisher(OdometryState, "/odometry", 10)

        self.get_logger().info("Started odometry node.")

    def encoder_callback(self, msg: CanReceiveSwerve):
        self.raw_encoders = msg
        self.last_encoders = self.get_clock().now()
        self.publish_odometry()

    def drive_callback(self, msg: CanReceiveDrive):
        self.drive_rpms = msg
        self.last_drive = self.get_clock().now()
        self.publish_odometry()

    def update_state(self):
        """Return body-frame velocity and yaw rate from the eight wheel readings."""
        velocity = Point()
        rpms = (
            self.drive_rpms.fl_rpm,
            self.drive_rpms.fr_rpm,
            self.drive_rpms.bl_rpm,
            self.drive_rpms.br_rpm,
        )
        encoders = (
            self.raw_encoders.fl_encoder,
            self.raw_encoders.fr_encoder,
            self.raw_encoders.bl_encoder,
            self.raw_encoders.br_encoder,
        )
        rot = 0.0
        for rpm, encoder, (x, y) in zip(rpms, encoders, WHEEL_LOCATIONS):
            # TODO: correct way to read encoder??
            angle = math.radians(encoder)
            speed = rpm * ERPM_TO_MPS
            fwd, left = speed * math.cos(angle), speed * math.sin(angle)
            velocity.x += fwd / 4.0
            velocity.y += left / 4.0
            rot += x * left - y * fwd
        omega = rot / (WHEELBASE**2 + TRACK_WIDTH**2)
        self.odometry = OdometryState(velocity=velocity, omega=omega)

    def publish_odometry(self):
        now = self.get_clock().now()
        if has_timed_out(self.last_drive, SENSOR_TIMEOUT, now) or has_timed_out(
            self.last_encoders, SENSOR_TIMEOUT, now
        ):
            return
        self.update_state()
        self.odometry_publisher_.publish(self.odometry)


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
