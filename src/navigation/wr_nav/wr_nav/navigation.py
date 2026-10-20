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
#           4 (TRAVERSAL)
#           5 (APPROACH)
#           6 (FOLLOWING)
#   '/person' (wr_detection_msgs/Person)
#       Description:
#           Gets the astronaut's position relative to a forward-facing camera.
#       Format:
#           position.x: right (meters, or pixels from image center when uncertain)
#           position.y: ignored
#           position.z: forward (meters)
#           gesture: ignored
#           confidence: confidence of person, determines when meters or pixels are used
#       Example:
#           position: {x: 0.0, y: 0.0, z: 2.0}
#           gesture: 1
#           confidence: 0.9
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
from wr_detection_msgs.msg import Person


class NavModes(Enum):
    STOP = 0
    DANCE = 1
    TRACKING = 2
    FINDING = 3
    TRAVERSAL = 4
    APPROACH = 5
    FOLLOWING = 6


class Nav(Node):
    DEFAULT_HEADING_TOLERANCE = 20.0  # degrees
    DEFAULT_DISTANCE_TOLERANCE = 1.0  # meters
    DEFAULT_ROTATION_KP = 0.5

    DANCE_LIMIT = 0.3  # maximum value to use for dancing
    GOOD_POSE_TIMEOUT = 1.0
    GOOD_PERSON_TIMEOUT = 1.0
    GOOD_APPROACH_TIMEOUT = 1.0
    GOOD_CONF_LEVEL = 0.60
    # TODO: should this be a rotation counter instead?
    FINDING_TIMEOUT = 10.0

    TRAVERSAL_SPEED = 0.3
    APPROACH_SPEED = 0.3
    FINDING_ROT = 0.3
    FAR_PERSON_CENTER_TOLERANCE = 40.0
    FAR_PERSON_ROTATION_KP = 0.005

    def __init__(self):
        super().__init__("nav")

        self.mode = NavModes.STOP  # not using change_mode is okay here
        self.traversal_target = None
        self.approach_target = None
        self.person = None
        self.last_dance_controls = (0.0, 0.0, 0.0)

        self.pose = None

        self.last_pose = None
        self.last_person = None
        self.last_approach_target = None
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

        self.traversal_target_subscriber_ = self.create_subscription(
            Point, "/target_gnss", self.traversal_target_callback, 10
        )
        self.approach_target_subscriber_ = self.create_subscription(
            Point, "/target_3d", self.approach_target_callback, 10
        )

        # ---------------------------------------------------------------------
        # Mode and Person Subscribers
        # ---------------------------------------------------------------------

        self.nav_mode_subscriber_ = self.create_subscription(
            Int32, "/nav_mode", self.nav_mode_callback, 10
        )
        self.person_subscriber_ = self.create_subscription(
            Person, "/person", self.person_callback, 10
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
        if self.mode == NavModes.TRAVERSAL:
            if self.last_pose is None or self.pose is None:
                self.publish_drive_helper(0.0, 0.0, 0.0)
                return
            # In case localization fails...
            if self.has_timed_out(self.last_pose, self.GOOD_POSE_TIMEOUT):
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
            case NavModes.TRAVERSAL:
                self.do_traversal()
            case NavModes.APPROACH:
                self.do_approach()
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

    def person_callback(self, msg: Person):
        self.person = msg
        self.last_person = self.get_clock().now()

    def traversal_target_callback(self, msg: Point):
        self.traversal_target = msg

    def approach_target_callback(self, msg: Point):
        self.approach_target = msg
        self.last_approach_target = self.get_clock().now()

    def do_finding(self):
        self.publish_drive_helper(0.0, 0.0, self.FINDING_ROT)
        # TODO: What to do here? What to pass to state machine?
        # Also, even if the robot finds the person, unless the state machine
        # updates the mode itself, the timeout will trigger as well.
        if self.has_timed_out(self.last_mode_change, self.FINDING_TIMEOUT):
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
        if self.person is None or self.has_timed_out(
            self.last_person, self.GOOD_PERSON_TIMEOUT
        ):
            self.get_logger().warning("wr_nav: cannot find person.")
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return
        elif self.person.confidence < self.GOOD_CONF_LEVEL:
            self.get_logger().warning(
                f"wr_nav: {self.person.confidence} confidence is not good enough to track"
            )
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        # x is probably cw
        pos = self.person.position
        if pos.z < 0.0:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return
        err = math.atan2(-pos.x, pos.z)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        self.publish_drive_helper(0.0, 0.0, rot)

    def do_traversal(self):
        """Navigates to a GNSS target by first facing the target and moving towards it."""
        if self.traversal_target is None:
            self.get_logger().warning("wr_nav: using traversal mode without a target.")
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return
        dpos = self.geo_to_dpos(
            Point(x=self.pose.x, y=self.pose.y), self.traversal_target
        )
        dist = math.hypot(dpos.x, dpos.y)

        if dist < self.dist_tolerance:
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        # Scale rotation by heading error.
        err = self.normalize_angle(math.atan2(dpos.y, dpos.x) - self.pose.theta)
        rot = self.clamp(self.kp_rot * err, -1.0, 1.0)
        if abs(err) < self.heading_tolerance:
            self.publish_drive_helper(self.TRAVERSAL_SPEED, 0.0, rot)
        else:
            self.publish_drive_helper(0.0, 0.0, rot)

    def do_following(self):
        if self.person is None or self.has_timed_out(
            self.last_person, self.GOOD_PERSON_TIMEOUT
        ):
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        pos = self.person.position
        if self.person.confidence < self.GOOD_CONF_LEVEL:
            rot = self.clamp(-self.FAR_PERSON_ROTATION_KP * pos.x, -1.0, 1.0)
            if abs(pos.x) < self.FAR_PERSON_CENTER_TOLERANCE:
                self.publish_drive_helper(self.APPROACH_SPEED, 0.0, rot)
            else:
                self.publish_drive_helper(0.0, 0.0, rot)
        else:
            self.drive_to_3d_target(pos)

    def do_approach(self):
        if self.approach_target is None or self.has_timed_out(
            self.last_approach_target, self.GOOD_APPROACH_TIMEOUT
        ):
            self.publish_drive_helper(0.0, 0.0, 0.0)
            return

        self.drive_to_3d_target(self.approach_target)

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
            self.publish_drive_helper(self.APPROACH_SPEED, 0.0, rot)
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

    def has_timed_out(self, last_update: Time | None, timeout: float) -> bool:
        """Checks if some update has exceeded its own timeout."""
        if last_update is None:
            return True
        since_last_update = (self.get_clock().now() - last_update).nanoseconds * 1e-9
        return since_last_update > timeout

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
