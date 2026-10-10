# =============================================================================
# Background:
# TODO: all of this
# TODO: implement drift correction
# =============================================================================
# Brief:
#
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
#       Format: TODO: Is this correctly documented?
#           x: longitude (degrees)
#           y: latitude (degrees)
#           theta: heading (radians)
#       Example:
#           x: 43.07
#           y: -89.4
#           theta: 39.2
#       Notes:
#           Theta can be a NaN! This means that localization is unsure of the
#           robot's current heading, and this doubles as a signal to the nav
#           node to move straight to imu correct. Also, theta should be CCW positive.
# =============================================================================

import rclpy
import math
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT, GpsFix


class Localization(Node):
    UBX_DEG_SCALE = 1e-7

    DEFAULT_OFF_DIST_THRESH = 20.0
    IMU_ERR_THRESH = math.radians(20.0)

    GOOD_FIX_TYPES = [
        GpsFix.GPS_FIX_2D,
        GpsFix.GPS_FIX_3D,
        GpsFix.GPS_PLUS_DEAD_RECKONING,
    ]
    GOOD_FIX_TIMEOUT = 2.0

    def __init__(self):
        super().__init__("localization")

        # This is NOT the initial position of the robot, this is just the position used for imu correction.
        self.init_pos = None
        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)

        self.imu_offset = None  # imu offset
        self.imu_raw = None  # last imu reading
        self.imu_ref = None  # used for checking if the robot is actually going straight

        self.last_good_fix = Time(seconds=0, clock_type=self.get_clock().clock_type)

        # Parameter(s)
        self.declare_parameter("off_dist_thresh", self.DEFAULT_OFF_DIST_THRESH)
        self.off_dist_threshold = (
            self.get_parameter("off_dist_thresh").get_parameter_value().double_value
        )

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

    def imu_callback(self, msg: Float32):
        self.imu_raw = math.radians(msg.data)
        if self.imu_ref is None:
            self.imu_ref = self.imu_raw
        if self.imu_offset is not None:
            # TODO: Make sure that this heading has the correct sign.
            self.pose.theta = self.normalize_angle(self.imu_raw - self.imu_offset)
        self.publish_pose()

    def gnss_callback(self, msg: UBXNavPVT):
        # Message might not have the information we want.
        if not msg.gnss_fix_ok or msg.gps_fix.fix_type not in self.GOOD_FIX_TYPES:
            return
        self.last_good_fix = self.get_clock().now()
        self.pose.x = msg.lon * self.UBX_DEG_SCALE
        self.pose.y = msg.lat * self.UBX_DEG_SCALE
        # This could be called from some explicit initialization event.
        # TODO: Validate that this resets reference pos if our imu changes too much.
        if self.init_pos is None or (
            (self.imu_ref is not None)
            and abs(self.normalize_angle(self.imu_raw - self.imu_ref))
            > self.IMU_ERR_THRESH
        ):
            self.init_pos = self.get_pos()
            self.imu_ref = self.imu_raw

        dpos = self.geo_to_dpos(self.init_pos, self.get_pos())
        # The IMU calibration triggers when more than 20 meters away. (By default)
        if math.hypot(dpos.x, dpos.y) > self.off_dist_threshold and (
            self.imu_raw is not None and self.imu_offset is None
        ):
            # Figure out imu offset.
            self.imu_offset = self.normalize_angle(
                self.imu_raw - math.atan2(dpos.y, dpos.x)
            )
            self.pose.theta = self.normalize_angle(self.imu_raw - self.imu_offset)
        self.publish_pose()

    def publish_pose(self):
        """Publish our current pose to /pose."""
        # Pose CAN publish a nan theta.
        # This must be accounted for by all of /pose's subscribers.
        since_last_good_fix = (
            self.get_clock().now() - self.last_good_fix
        ).nanoseconds * 1e-9
        if (
            not math.isnan(self.pose.x)
            and not math.isnan(self.pose.y)
            and since_last_good_fix < self.GOOD_FIX_TIMEOUT
        ):
            self.pose_publisher_.publish(self.pose)

    def get_pos(self) -> Point:
        """Get position of robot as a Point."""
        return Point(x=self.pose.x, y=self.pose.y)

    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        """
        Takes two different geographic coordinates and spits out a difference in meters.
        There is an error of approximately 1-3m depending on direction after 1km of travel.
        """
        EARTH_RADIUS = 6371000.0

        dlat = math.radians(end.y - start.y)
        dlon = math.radians(end.x - start.x)
        # Latitude midpoint
        mid_lat = math.radians((start.y + end.y) / 2)
        return Point(x=dlon * EARTH_RADIUS * math.cos(mid_lat), y=dlat * EARTH_RADIUS)

    # TODO: Should we use this version instead? Currently unused.
    def geo_to_dpos_wgs84(self, start: Point, end: Point) -> Point:
        """
        Takes two different geographic coordinates and spits out a difference in meters.
        This version of geo_to_dpos takes into account that the Earth is more like an
        oblate spheroid, hence some slightly confusing math to compute effective radii along
        the meridional and transverse planes is included. (How the derivation for these radii works
        is beyond me.)

        This version is more accurate than `geo_to_dpos`, having millimeters of error after 1km of travel.
        """
        a = 6378137.0  # semi-major axis in meters
        e2 = 0.00669437999014  # first eccentricity squared

        mid_lat = math.radians((start.y + end.y) / 2.0)
        dlat = self.normalize_angle(math.radians(end.y - start.y))
        dlon = self.normalize_angle(math.radians(end.x - start.x))

        # What the fuck?
        denom = math.sqrt(1.0 - e2 * math.sin(mid_lat) ** 2)
        M = a * (1.0 - e2) / (denom**3)  # Meridional (y)
        N = a / denom  # Transverse (x)

        return Point(x=dlon * N * math.cos(mid_lat), y=dlat * M)

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
