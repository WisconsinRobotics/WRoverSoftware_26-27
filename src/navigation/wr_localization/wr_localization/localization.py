# =============================================================================
# Background:
# TODO: all of this
# =============================================================================
# Brief:
#
# =============================================================================
# Subscribes to:
#   '/imu'
#   '/ubx_nav_pvt'
#
# =============================================================================
# Publishes to:
#   '/pose' (geometry_msgs/Pose2D)
# =============================================================================

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose2D

from ublox_ubx_msgs.msg import UBXNavPVT


class Localization(Node):
    UBX_DEG_SCALE = 1e-7
    EARTH_RADIUS = 6371000.0
    DEFAULT_OFF_DIST_THRESH = 10.0

    def __init__(self):
        super().__init__("localization")

        self.init_pos = None
        self.pose = Pose2D(x=math.nan, y=math.nan, theta=math.nan)
        self.imu_offset = None
        self.imu_raw = None

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
        self.imu_raw = msg.data
        if self.imu_offset is not None:
            # TODO: Make sure that this heading has the correct sign.
            self.pose.theta = self.normalize_angle(msg.data - self.imu_offset)
        self.publish_pose()

    def gnss_callback(self, msg: UBXNavPVT):
        # Message might not have the information we want.
        if not msg.gnss_fix_ok or msg.gps_fix.fix_type not in [2, 3, 4]:
            return
        current_geo = Point(
            x=msg.lon * self.UBX_DEG_SCALE, y=msg.lat * self.UBX_DEG_SCALE
        )
        # This could be called from some explicit initialization event.
        if self.init_pos is None:
            self.init_pos = current_geo
        dpos = self.geo_to_dpos(self.init_pos, current_geo)
        self.pose.x = dpos.x
        self.pose.y = dpos.y
        # TODO: This condition is just a placeholder. It triggers when more than 10 meters away.
        # Since we have IMU data, we could use that to verify that we kept our heading while moving.
        if math.hypot(dpos.x, dpos.y) > self.off_dist_threshold and (
            self.imu_raw is not None and self.imu_offset is None
        ):
            # Figure out imu offset.
            self.imu_offset = self.normalize_angle(
                self.imu_raw - math.atan2(dpos.y, dpos.x)
            )
        self.publish_pose()

    def publish_pose(self):
        # TODO: Figure out if pose should publish without heading data.
        # This must be accounted for by all of /pose's subscribers.
        if (
            not math.isnan(self.pose.x)
            and not math.isnan(self.pose.y)
            and not math.isnan(self.pose.theta)
        ):
            self.pose_publisher_.publish(self.pose)

    # Takes two different geographic coordinates and spits out a difference in meters.
    def geo_to_dpos(self, start: Point, end: Point) -> Point:
        dlat = math.radians(end.y - start.y)
        dlon = math.radians(end.x - start.x)
        return Point(
            x=dlon * self.EARTH_RADIUS * math.cos(math.radians(start.y)),
            y=dlat * self.EARTH_RADIUS,
        )

    # Binds the angle from -pi to pi.
    def normalize_angle(self, angle: float) -> float:
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
