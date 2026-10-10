# =============================================================================
# Background:
# TODO: all of this
# =============================================================================
# Brief:
#
# =============================================================================
# Subscribes to:
#   '/pose' (geometry_msgs/Pose2D)
#       Description:
#           Publishes our geographic coordinates and heading as a pose.
#       Format: TODO: Is this correctly documented?
#           x: longitude (degrees)
#           y: latitude (degrees)
#           theta: heading (radians)
#       Example:
#           x: 43.07
#           y: -89.4
#           theta: 39.2
#   '/target' (geometry_msgs/Point)
#      Takes (lon, lat).
#
# =============================================================================
# Publishes to:
#   '/swerve' (std_msgs/Float32MultiArray)
#       Description:
#           Desired rover motion, expressed with forward, strafe, and left/right trigger.
#       Format:
#           [forward, strafe, left_trigger, right_trigger]
#       Example:
#           # Idle
#           [0.0, 0.0, 1.0, 1.0]
#           # Forward
#           [1.0, 0.0, 1.0, 1.0]
# =============================================================================

import math
import random
import rclpy
from rclpy.time import Time
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose2D


class NavModes(Enum):
    IDLE = 0
    DANCE = 1
    IMU_CORRECTION = 2
    GNSS_TARGET = 3
    FOLLOW_PATH = 4


class Nav(Node):
    DEFAULT_HEADING_TOLERANCE = 20.0  # degrees
    DEFAULT_DISTANCE_TOLERANCE = 1.0  # meters
    DEFAULT_ROTATION_KP = 0.5

    DANCE_LIMIT = 0.3  # maximum value to use for dancing
    GOOD_POSE_TIMEOUT = 1.0

    def __init__(self):
        super().__init__("nav")

        self.mode = NavModes.IDLE  # not using change_mode is okay here
        self.target = None
        self.last_dance_controls = (0.0, 0.0, 0.0)

        self.last_pose = Time(seconds=0, clock_type=self.get_clock().clock_type)

        # Parameter(s)
        self.declare_parameter("heading_tolerance", self.DEFAULT_HEADING_TOLERANCE)
        self.heading_tolerance = math.radians(
            self.get_parameter("heading_tolerance").get_parameter_value().double_value
        )
        self.declare_parameter("dist_tolerance", self.DEFAULT_DISTANCE_TOLERANCE)
        self.dist_tolerance = (
            self.get_parameter("dist_tolerance").get_parameter_value().double_value
        )
        self.declare_parameter("kp_rot", self.DEFAULT_ROTATION_KP)
        self.kp_rot = self.get_parameter("kp_rot").get_parameter_value().double_value

        # ---------------------------------------------------------------------
        # Pose Subscriber
        # ---------------------------------------------------------------------

        self.pose_subscriber_ = self.create_subscription(
            Pose2D, "/pose", self.pose_callback, 10
        )

        # ---------------------------------------------------------------------
        # Target Subscriber
        # ---------------------------------------------------------------------

        self.target_subscriber_ = self.create_subscription(
            Point, "/target", self.target_callback, 10
        )

        # ---------------------------------------------------------------------
        # Swerve Publisher
        # ---------------------------------------------------------------------

        self.swerve_publisher_ = self.create_publisher(Float32MultiArray, "/swerve", 10)

        # ---------------------------------------------------------------------
        # Failsafe Timer
        # ---------------------------------------------------------------------
        self.failsafe_timer_ = self.create_timer(1.0, self.failsafe_callback)

    def pose_callback(self, msg: Pose2D):
        # If we don't have a heading, we assume we have to get it. So, drive forward.
        if math.isnan(msg.theta) and self.mode != NavModes.IDLE:
            self.change_mode(NavModes.IMU_CORRECTION)
        elif not math.isnan(msg.theta) and self.mode == NavModes.IMU_CORRECTION:
            # Got a heading, so resume (or go idle if there's no target)
            # TODO: should we go back to our previous mode instead of guessing what to do?
            self.change_mode(
                NavModes.GNSS_TARGET if self.target is not None else NavModes.IDLE
            )

        self.last_pose = self.get_clock().now()

        match self.mode:
            case NavModes.IDLE:
                self.publish_drive(0.0, 0.0, 0.0)
            case NavModes.DANCE:
                self.do_dance()
            case NavModes.IMU_CORRECTION:
                self.publish_drive(1.0, 0.0, 0.0)
            case NavModes.GNSS_TARGET:
                self.do_gnss_target(msg)
            case NavModes.FOLLOW_PATH:
                pass  # TODO: don't yet have information to do this

    def target_callback(self, msg: Point):
        self.target = msg
        self.change_mode(NavModes.GNSS_TARGET)

    # Failsafe in case localization somehow fails...
    # TODO: recover last working mode on pose recovery
    def failsafe_callback(self):
        since_last_pose = (self.get_clock().now() - self.last_pose).nanoseconds * 1e-9
        if since_last_pose > self.GOOD_POSE_TIMEOUT:
            if self.mode != NavModes.IDLE:
                self.get_logger().warning("wr_nav: no pose received in time, idling.")
            self.change_mode(NavModes.IDLE)
            self.publish_drive(0.0, 0.0, 0.0)

    # TODO: Decide if we should use this...
    def do_dance(self):
        """The idea behind "dancing" is so that we can wiggle ourself out of any tight spots."""
        self.last_dance_controls = tuple(
            self.clamp(
                c + random.choice([0.05, -0.05]), -self.DANCE_LIMIT, self.DANCE_LIMIT
            )
            for c in self.last_dance_controls
        )
        self.publish_drive(*self.last_dance_controls)

    def do_gnss_target(self, pose: Pose2D):
        if self.target is None:  # This should NEVER occur, but just in case...
            self.get_logger().warning(
                "wr_nav: using gnss target mode without a target."
            )
            self.change_mode(NavModes.IDLE)
            return
        # Basic implementation: face target, then move towards it.
        dpos = self.geo_to_dpos(Point(x=pose.x, y=pose.y), self.target)
        dist = math.hypot(dpos.x, dpos.y)

        if dist < self.dist_tolerance:  # Close enough to target.
            self.change_mode(NavModes.IDLE)
            self.target = None
            self.publish_drive(0.0, 0.0, 0.0)
            return

        # Scale rotation by heading error.
        err = self.normalize_angle(math.atan2(dpos.y, dpos.x) - pose.theta)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        if abs(err) < self.heading_tolerance:
            self.publish_drive(1.0, 0.0, rot)
        else:
            self.publish_drive(0.0, 0.0, rot)

    def change_mode(self, mode: NavModes):
        """Changes navigation mode."""
        if mode == NavModes.DANCE:
            self.last_dance_controls = (0.0, 0.0, 0.0)
        self.mode = mode

    def publish_drive(self, fwd: float, swerve: float, rot: float):
        """Publish swerve drive controls to /swerve."""
        arr = Float32MultiArray(data=[fwd, swerve, rot, 0.0])
        self.swerve_publisher_.publish(arr)

    # The following two functions are borrowed code from localization.
    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        """
        Takes two different geographic coordinates and spits out a difference in meters.
        There is an error of approximately a meter after 1km of travel.
        """
        EARTH_RADIUS = 6371000.0

        dlat = math.radians(end.y - start.y)
        dlon = math.radians(end.x - start.x)
        # Latitude midpoint
        mid_lat = math.radians((start.y + end.y) / 2)
        return Point(x=dlon * EARTH_RADIUS * math.cos(mid_lat), y=dlat * EARTH_RADIUS)

    def normalize_angle(self, angle: float) -> float:
        """Binds the angle from -pi to pi."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, val, low, high):
        """Clamps val between low and high."""
        return max(low, min(high, val))


# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = Nav()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
