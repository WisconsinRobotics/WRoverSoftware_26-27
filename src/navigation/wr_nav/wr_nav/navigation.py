# =============================================================================
# Background:
# TODO: all of this
# =============================================================================
# Brief:
#
# =============================================================================
# Subscribes to:
#   '/pose' (geometry_msgs/Pose2D)
#   '/target' (???)
#
# =============================================================================
# Publishes to:
#   '/swerve' (std_msgs/Float32MultiArray)
# =============================================================================

import rclpy
import math
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose2D


class NavModes(Enum):
    IDLE = 0
    DANCE = 1
    IMU_CORRECTION = 2
    GNSS_TARGET = 3


class Nav(Node):
    def __init__(self):
        super().__init__("localization")

        self.mode = NavModes.IDLE

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
            Point, "/target", self.pose_callback, 10
        )

        # ---------------------------------------------------------------------
        # Swerve Publisher
        # ---------------------------------------------------------------------

        self.swerve_publisher_ = self.create_publisher(Float32MultiArray, "/swerve", 10)

    def pose_callback(self, msg: Pose2D):
        # TODO: If we don't have a heading, we assume we have to get it. So, drive forward.
        if math.isnan(msg.theta):
            self.publish_drive(1.0, 0.0, 0.0)
            return
        # TODO: Figure out modes

    def publish_drive(self, fwd: float, swerve: float, rot: float):
        self.swerve_publisher_.publish([fwd, swerve, 0.0, rot])


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
