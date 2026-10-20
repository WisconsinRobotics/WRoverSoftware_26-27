# =============================================================================
# Background:
#   Autonomous navigation uses the rover's geographic position and heading to
#   drive toward a GNSS target. Localization supplies longitude, latitude, and
#   heading, while the swerve drive pipeline turns motion commands into wheel
#   speed and steering commands.
#
#   This node approximates the displacement to the target in east/north meters
#   to calculate the remaining distance and desired heading. It turns toward
#   the target before driving forward, continues correcting its heading while
#   moving, and stops once it reaches the configured distance tolerance.
#
# =============================================================================
# Brief:
#   Uses the rover's pose to steer toward a GNSS target and publishes swerve
#   drive commands.
# =============================================================================
# Subscribes to:
#   '/pose' (geometry_msgs/Pose2D)
#       Description:
#           Retrieves our geographic coordinates and heading as a pose.
#       Format:
#           x: latitude (degrees)
#           y: longitude (degrees)
#           theta: heading (radians)
#       Example:
#           x: 43.07
#           y: -89.4
#           theta: 39.2
#   '/target_gnss' (geometry_msgs/Point)
#       Description:
#           Sets our current target GNSS coordinate.
#       Format:
#           x: latitude (degrees)
#           y: longitude (degrees)
#           z: ignored
#       Example:
#           x: 43.07
#           y: -89.4
#           z: 0.0
#   '/target_3d' (geometry_msgs/Point)
#       Description:
#           Sets a target relative to the forward-facing camera.
#       Format:
#           x: right (meters)
#           y: down (meters, ignored)
#           z: forward (meters)
#   '/nav_mode' (std_msgs/Int32)
#       Description:
#           Sets the current navigation mode.
#       Example:
#           0 (STOP)
#           2 (TRACKING)
#           4 (GNSS_TARGET)
#           5 (TARGET_3D)
#           6 (FOLLOWING)
#   '/person' (wr_gesture_detection/Person)
#       Description:
#           Gets the astronaut's position relative to a forward-facing camera.
#       Format:
#           position.x: right (meters OR pixels when far)
#           position.y: down (meters, ignored)
#           position.z: forward (meters)
#           gesture: observed gesture code (ignored for now)
#           far: indication for when using pixels instead of meters
#       Example:
#           position: {x: 0.0, y: 0.0, z: 2.0}
#           gesture: 1
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
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Point, Pose2D
from wr_gesture_detection.Person import Person
from wr_gesture_detection.gesture import Gesture


class NavModes(Enum):
    STOP = 0
    DANCE = 1
    TRACKING = 2
    FINDING = 3
    GNSS_TARGET = 4
    TARGET_3D = 5
    FOLLOWING = 6


class Nav(Node):
    DEFAULT_HEADING_TOLERANCE = 20.0  # degrees
    DEFAULT_DISTANCE_TOLERANCE = 1.0  # meters
    DEFAULT_ROTATION_KP = 0.5

    DANCE_LIMIT = 0.3  # maximum value to use for dancing
    GOOD_POSE_TIMEOUT = 1.0
    GOOD_PERSON_TIMEOUT = 1.0
    GOOD_TARGET_3D_TIMEOUT = 1.0
    # TODO: should this be a rotation counter instead?
    FINDING_TIMEOUT = 10.0

    GNSS_TARGET_SPEED = 0.3
    TARGET_3D_SPEED = 0.3
    FINDING_ROT = 0.3
    FAR_PERSON_CENTER_TOLERANCE = 40.0
    FAR_PERSON_ROTATION_KP = 0.005

    def __init__(self):
        super().__init__("nav")

        self.mode = NavModes.STOP  # not using change_mode is okay here
        self.gnss_target = None
        self.person_target = None
        self.target_3d = None
        self.last_dance_controls = (0.0, 0.0, 0.0)

        self.pose = None

        self.last_pose = None
        self.last_person = None
        self.last_target_3d = None
        # Thankfully we don't NEED to set this one to None.
        self.last_mode_change = Time(seconds=0, clock_type=self.get_clock().clock_type)

        # Read startup values directly from the declared parameters, including overrides.
        self.heading_tolerance = math.radians(
            self.declare_parameter("heading_tolerance", self.DEFAULT_HEADING_TOLERANCE)
            .get_parameter_value()
            .double_value
        )
        self.dist_tolerance = (
            self.declare_parameter("dist_tolerance", self.DEFAULT_DISTANCE_TOLERANCE)
            .get_parameter_value()
            .double_value
        )
        self.kp_rot = (
            self.declare_parameter("kp_rot", self.DEFAULT_ROTATION_KP)
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------------------
        # Pose Subscriber
        # ---------------------------------------------------------------------

        self.pose_subscriber_ = self.create_subscription(
            Pose2D, "/pose", self.pose_callback, 10
        )

        # ---------------------------------------------------------------------
        # Target Subscribers
        # ---------------------------------------------------------------------

        self.target_subscriber_ = self.create_subscription(
            Point, "/target_gnss", self.target_callback, 10
        )
        self.target_3d_subscriber_ = self.create_subscription(
            Point, "/target_3d", self.target_3d_callback, 10
        )

        # ---------------------------------------------------------------------
        # Mode and Person Subscribers
        # ---------------------------------------------------------------------

        self.nav_mode_subscriber_ = self.create_subscription(
            Int32, "/nav_mode", self.nav_mode_callback, 10
        )
        self.person_subscriber_ = self.create_subscription(
            # Person, "/person", self.person_callback, 10
            Point,
            "/person",
            self.person_callback,
            10,
        )

        # ---------------------------------------------------------------------
        # Swerve Publisher
        # ---------------------------------------------------------------------

        self.swerve_publisher_ = self.create_publisher(Float32MultiArray, "/swerve", 10)

        # ---------------------------------------------------------------------
        # Main Nav Loop at 10Hz
        # ---------------------------------------------------------------------
        self.main_timer_ = self.create_timer(0.1, self.main_timer_callback)

        self.get_logger().info("Started nav node.")

    def pose_callback(self, msg: Pose2D):
        self.last_pose = self.get_clock().now()
        self.pose = msg

    def main_timer_callback(self):
        if self.mode == NavModes.GNSS_TARGET:
            if self.last_pose is None or self.pose is None:
                self.publish_drive_helper(0.0, 0.0, 0.0)
                return
            # In case localization fails...
            since_last_pose = (
                self.get_clock().now() - self.last_pose
            ).nanoseconds * 1e-9
            if since_last_pose > self.GOOD_POSE_TIMEOUT:
                self.get_logger().warning("wr_nav: no pose received in time, idling.")
                # Ideally, navigation should never change its own mode.
                # This should be passed to state machine.
                self.change_mode(NavModes.STOP)
        match self.mode:
            case NavModes.STOP:
                self.publish_drive_helper(0.0, 0.0, 0.0)
            case NavModes.DANCE:
                self.do_dance()
            case NavModes.TRACKING:
                self.do_tracking()
            case NavModes.FINDING:
                self.do_finding()
            case NavModes.GNSS_TARGET:
                self.do_gnss_target()
            case NavModes.TARGET_3D:
                self.do_3d_target()
            case NavModes.FOLLOWING:
                self.do_following()

    def nav_mode_callback(self, msg: Int32):
        """Changes our current navigation mode."""
        try:
            mode = NavModes(msg.data)
        except ValueError:
            self.get_logger().warning(f"wr_nav: unknown navigation mode {msg.data}.")
            return
        self.change_mode(mode)

    # TODO: Make person a message type so that this can work.
    # def person_callback(self, msg: Person):
    def person_callback(self, msg: Point):
        # self.person_target = msg
        self.person_target = Person(msg, Gesture.NONE)
        self.last_person = self.get_clock().now()

    def target_callback(self, msg: Point):
        self.gnss_target = msg

    def target_3d_callback(self, msg: Point):
        self.target_3d = msg
        self.last_target_3d = self.get_clock().now()

    def do_finding(self):
        self.publish_drive_helper(0.0, 0.0, self.FINDING_ROT)
        # TODO: What to do here? What to pass to state machine?
        # Also, even if the robot finds the person, unless the state machine
        # updates the mode itself, the timeout will trigger as well.
        since_last_mode_change = (
            self.get_clock().now() - self.last_mode_change
        ).nanoseconds * 1e-9
        if since_last_mode_change > self.FINDING_TIMEOUT:
            self.get_logger().warning("wr_nav: exceeding finding timeout.")

    # TODO: Decide if we should use this...
    def do_dance(self):
        """The idea behind "dancing" is to wiggle ourself out of any tight spots."""
        self.last_dance_controls = tuple(
            self.clamp(
                c + random.choice([0.05, -0.05]), -self.DANCE_LIMIT, self.DANCE_LIMIT
            )
            for c in self.last_dance_controls
        )
        self.publish_drive_helper(*self.last_dance_controls)

    # TODO: This probably doesn't work
    def do_tracking(self):
        """Rotate in place to face the person relative to the camera."""
        if self.person_target is None or self.last_person is None:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        since_last_person = (
            self.get_clock().now() - self.last_person
        ).nanoseconds * 1e-9
        if since_last_person > self.GOOD_PERSON_TIMEOUT:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        # x is probably cw
        pos = self.person_target.position
        if (pos.z < 0.0):
            self.publish_drive_helper(0.0, 0.0, 0.0)
        err = math.atan2(-pos.x, pos.z)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        self.publish_drive_helper(0.0, 0.0, rot)

    def do_gnss_target(self):
        """Navigates to a GNSS target by first facing the target and moving towards it."""
        if self.gnss_target is None:  # This should NEVER occur, but just in case...
            self.get_logger().warning(
                "wr_nav: using gnss target mode without a target."
            )
            self.change_mode(NavModes.STOP)
            return
        dpos = self.geo_to_dpos(Point(x=self.pose.x, y=self.pose.y), self.gnss_target)
        dist = math.hypot(dpos.x, dpos.y)

        if dist < self.dist_tolerance:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        # Scale rotation by heading error.
        err = self.normalize_angle(math.atan2(dpos.y, dpos.x) - self.pose.theta)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        if abs(err) < self.heading_tolerance:
            self.publish_drive_helper(self.GNSS_TARGET_SPEED, 0.0, rot)
        else:
            self.publish_drive_helper(0.0, 0.0, rot)

    def do_following(self):
        if self.person_target is None or self.last_person is None:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return
        since_last_person = (
            self.get_clock().now() - self.last_person
        ).nanoseconds * 1e-9
        if since_last_person > self.GOOD_PERSON_TIMEOUT:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        pos = self.person_target.position
        # Using getattr since Person doesn't have far just yet.
        if getattr(self.person_target, "far", False):
            rot = self.clamp(-self.FAR_PERSON_ROTATION_KP * pos.x, -1.0, 1.0)
            self.publish_drive_helper(
                self.TARGET_3D_SPEED
                if abs(pos.x) < self.FAR_PERSON_CENTER_TOLERANCE else 0.0,
                0.0, rot)
        else:
            self.drive_to_3d_target(pos)

    def do_3d_target(self):
        if self.target_3d is None or self.last_target_3d is None:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        since_last_target = (
            self.get_clock().now() - self.last_target_3d
        ).nanoseconds * 1e-9
        if since_last_target > self.GOOD_TARGET_3D_TIMEOUT:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        self.drive_to_3d_target(self.target_3d)

    # TODO: See if this works, just a rough draft of a function
    def drive_to_3d_target(self, target: Point):
        # x is right, z is forward.
        right = target.x
        forward = target.z

        if math.hypot(right, forward) < self.dist_tolerance:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        # x is CW positive
        err = math.atan2(-right, forward)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        if abs(err) < self.heading_tolerance:
            self.publish_drive_helper(self.TARGET_3D_SPEED, 0.0, rot)
        else:
            self.publish_drive_helper(0.0, 0.0, rot)

    def change_mode(self, mode: NavModes):
        """Changes navigation mode."""
        if mode == NavModes.DANCE:
            self.last_dance_controls = (0.0, 0.0, 0.0)
        self.last_mode_change = self.get_clock().now()
        self.mode = mode

    def publish_drive_helper(self, fwd: float, strafe: float, rot: float):
        """Publish swerve drive controls to /swerve."""
        arr = Float32MultiArray(data=[fwd, strafe, rot, 0.0])
        self.swerve_publisher_.publish(arr)

    def destroy_node(self):
        """Publish zero drive before shutdown."""
        try:
            if self.context.ok():
                self.publish_drive_helper(0.0, 0.0, 0.0)
        finally:
            super().destroy_node()

    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        """
        Takes two different geographic coordinates and spits out a difference in meters.
        There is an error of approximately 1-3m depending on direction after 1km of travel.
        """
        EARTH_RADIUS = 6371000.0

        dlat = math.radians(end.x - start.x)
        dlon = math.radians(end.y - start.y)
        # Latitude midpoint
        mid_lat = math.radians((start.x + end.x) / 2)
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
