# =============================================================================
# Background:
#   The Xbox controller is connected to the basestation, which runs this node.
#   This node converts joystick input into swerve commands.
#
#   The controller uses squared stick input to provide finer control near the
#   center of the joystick while retaining full output at the limits.
#
# =============================================================================
# Brief:
#   This ROS 2 node implements the first stage of the swerve drive pipeline:
#     1. Reads Xbox controller input through pygame.
#     2. Applies a deadband to remove joystick jitter.
#     3. Squares joystick input while preserving its sign.
#     4. Handles controller connection and disconnection.
#     5. Publishes rover motion commands at 20 Hz.
#
# =============================================================================
# Subscribes to:
#   None
#
# =============================================================================
# Publishes to:
#   '/swerve' (std_msgs/Float32MultiArray)
#       Description:
#           Desired rover motion from the Xbox controller,
#           expressed with forward, strafe, and left/right trigger.
#       Format:
#           [forward, strafe, left_trigger, right_trigger]
#       Example:
#           # Idle
#           [0.0, 0.0, -1.0, -1.0]
#           # Forward
#           [1.0, 0.0, -1.0, -1.0]
#
# =============================================================================

import math

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# =============================================================================
# Controller configuration
# =============================================================================

PUBLISH_PERIOD = 0.05
AXIS_DEADBAND = 0.05

# =============================================================================
# Xbox controller node
# =============================================================================


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")

        # ---------------------------------------------------------------------
        # ROS publisher
        # ---------------------------------------------------------------------

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            "swerve",
            10,
        )

        # ---------------------------------------------------------------------
        # Controller configuration
        # ---------------------------------------------------------------------

        self.axis_deadband = AXIS_DEADBAND

        # Store pygame joystick objects in a consistent list.
        self.joysticks = []

        # Initialize pygame and the joystick subsystem.
        pygame.init()
        pygame.joystick.init()

        # Detect controllers that were already connected when the node started.
        self.update_joysticks()

        # ---------------------------------------------------------------------
        # Publish controller commands at 20 Hz
        # ---------------------------------------------------------------------

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish,
        )

        self.get_logger().info("Started drive controller")

    # =========================================================================
    # Controller management
    # =========================================================================

    def update_joysticks(self):
        """
        Refresh the list of connected pygame joysticks.
        """

        self.joysticks = [
            pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())
        ]

        for joystick in self.joysticks:
            if not joystick.get_init():
                joystick.init()

        self.get_logger().info(f"Connected controllers: {len(self.joysticks)}")

    # =========================================================================
    # Controller input
    # =========================================================================

    def get_axis(self, joystick, axis):
        """
        Read an axis and apply a squared response while preserving its sign.

        This provides finer control around the joystick center while retaining
        full output at the limits.
        """

        value = joystick.get_axis(axis)

        return math.copysign(value * value, value)

    def apply_deadband(self, motion):
        """
        Zero small joystick/trigger values to remove controller jitter.
        """

        for i in range(len(motion)):
            if abs(motion[i]) < self.axis_deadband:
                motion[i] = 0.0

        return motion

    def get_motion(self):
        """
        Convert Xbox controller input into the /swerve command format.

        Returns:
            [forward, strafe, left_trigger, right_trigger]
        """

        joystick = self.joysticks[0]

        motion = [
            # Left stick Y:
            # pygame reports forward as negative, so invert it.
            -self.get_axis(joystick, 1),
            # Right stick Y:
            # Used as the rover strafe command.
            self.get_axis(joystick, 3),
            # Left trigger.
            joystick.get_axis(2),
            # Right trigger.
            joystick.get_axis(5),
        ]

        return self.apply_deadband(motion)

    # =========================================================================
    # ROS publisher
    # =========================================================================

    def publish(self):
        """
        Process pygame events and publish the latest controller command.
        """

        self.handle_events()

        # No controller connected.
        if not self.joysticks:
            return

        motion = self.get_motion()

        msg = Float32MultiArray()
        msg.data = motion

        self.publisher_.publish(msg)

    # =========================================================================
    # Pygame event handling
    # =========================================================================

    def handle_events(self):
        """
        Handle controller connection, disconnection, and button events.
        """

        for event in pygame.event.get():
            # -----------------------------------------------------------------
            # Pygame window closed
            # -----------------------------------------------------------------

            if event.type == pygame.QUIT:
                self.stop_robot()
                rclpy.shutdown()
                return

            # -----------------------------------------------------------------
            # Controller connected
            # -----------------------------------------------------------------

            if event.type == pygame.JOYDEVICEADDED:
                self.update_joysticks()

            # -----------------------------------------------------------------
            # Controller disconnected
            # -----------------------------------------------------------------

            elif event.type == pygame.JOYDEVICEREMOVED:
                self.stop_robot()

                self.joysticks = []

                self.get_logger().warn(f"Joystick {event.instance_id} disconnected")

            # -----------------------------------------------------------------
            # Controller button
            # -----------------------------------------------------------------

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.joy == 0 and event.button == 2:
                    self.get_logger().info("Pressed first controller (DRIVE)")

    # =========================================================================
    # Safety
    # =========================================================================

    def stop_robot(self):
        """
        Immediately publish a neutral swerve command.

        Trigger axes are represented by -1.0 when released, matching the
        pygame Xbox controller convention used by this node.
        """

        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, -1.0, -1.0]

        self.publisher_.publish(msg)

    # =========================================================================
    # Shutdown
    # =========================================================================

    def destroy_node(self):
        """
        Stop the rover and clean up pygame before destroying the ROS node.
        """

        self.stop_robot()

        pygame.quit()

        super().destroy_node()


# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = DriveController()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
