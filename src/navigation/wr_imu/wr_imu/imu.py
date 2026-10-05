# =============================================================================
# Background:
#   The CTRE Phoenix6 Pigeon2 IMU measures yaw using a gyroscope.
#   It communicates over the CAN bus. The CAN bus is hardcoded to "can0",
#   and the Pigeon2's device ID is hardcoded to 10.
#
#   The Pigeon2's yaw can accumulate drift over time. This node does not
#   correct the Pigeon2's internal yaw reading. Instead, it publishes the
#   current raw yaw so that localization can combine it with GNSS information
#   and correct for accumulated drift.
#
# =============================================================================
# Brief:
#   This ROS 2 node publishes the Pigeon2 yaw at 100 Hz:
#     1. Reads the current yaw from the Pigeon2 over CAN.
#     2. Publishes the yaw as a std_msgs/Float32.
#     3. Downstream heading nodes use the IMU yaw together with GNSS information
#        to produce a drift-corrected heading.
#
# =============================================================================
# Subscribes to:
#   None
#
# =============================================================================
# Publishes to:
#   '/imu' (std_msgs/Float32)
#       Description:
#           Raw Pigeon2 yaw in degrees, published at 100 Hz.
#       Format:
#           yaw (float32)
#       Example:
#           180.0
#       Notes:
#           The Pigeon2 yaw is continuous and does not necessarily wrap at
#           360 degrees. For example, after multiple rotations it may report
#           450 degrees. Consumers that require a [0, 360) representation
#           should perform the appropriate modulo operation.
#
# =============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from phoenix6.hardware import Pigeon2


class Imu(Node):
    def __init__(self):
        super().__init__("imu")

        # ---------------------------------------------------------------------
        # Pigeon2
        # ---------------------------------------------------------------------

        self.pigeon2 = Pigeon2(10, "can0")

        # ---------------------------------------------------------------------
        # Yaw status signal, configured to update at 100 Hz
        # ---------------------------------------------------------------------

        self.yaw_signal = self.pigeon2.get_yaw()
        self.yaw_signal.set_update_frequency(100.0)

        # ---------------------------------------------------------------------
        # Yaw publisher
        # ---------------------------------------------------------------------

        self.publisher_ = self.create_publisher(Float32, "/imu", 10)

        # ---------------------------------------------------------------------
        # Publish yaw at 100 Hz
        # ---------------------------------------------------------------------

        self.timer_ = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("Started IMU node")

    # =========================================================================
    # Timer callback
    # =========================================================================

    def timer_callback(self):
        # ---------------------------------------------------------------------
        # Refresh the Pigeon2 yaw signal
        # ---------------------------------------------------------------------

        self.yaw_signal.refresh()

        # ---------------------------------------------------------------------
        # Read yaw in degrees
        # ---------------------------------------------------------------------

        yaw = self.yaw_signal.value

        # ---------------------------------------------------------------------
        # Publish yaw
        # ---------------------------------------------------------------------

        message = Float32()
        message.data = float(yaw)

        self.publisher_.publish(message)


# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = Imu()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
