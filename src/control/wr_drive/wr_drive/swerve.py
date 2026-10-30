# =============================================================================
# Background:
#   Swerve drive uses four independently driven and steered wheel modules.
#   We use IK (inverse kinematics) to control the wheel motors.
#
#   The VESCs (motor controllers) receive commands over the CAN bus to control
#   wheel speed and steering position. They also broadcast encoder feedback over CAN.
#
# =============================================================================
# Brief:
#   This ROS 2 node implements a swerve drive controller with IK:
#     1. Receives rover motion commands.
#     2. Computes the desired speed and angle for each wheel.
#     3. Applies wheel angle wrapping.
#     4. Applies acceleration/deceleration limiting.
#     5. Uses steering encoder feedback to correct wheel-angle drift.
#     6. Converts the four wheel commands into VESC CAN command strings.
#     7. Publishes all eight CAN commands as one newline-separated String.
#
# =============================================================================
# Subscribes to:
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
#
#   '/can_receive_swerve' (CanReceiveSwerve)
#       Description:
#           Absolute encoder positions of swerve motors, in degrees, using the
#           VESC convention: 180.0 is straight ahead, valid range is 135-225.
#       Format:
#           fl_encoder (float32)
#           fr_encoder (float32)
#           bl_encoder (float32)
#           br_encoder (float32)
#       Example:
#           # Driving straight forward
#           fl_encoder: 180.0
#           fr_encoder: 180.0
#           bl_encoder: 180.0
#           br_encoder: 180.0
#
# =============================================================================
# Publishes to:
#   '/can_send' (std_msgs/String)
#       Description:
#           VESC command strings to be sent over the CAN bus by wr_can.
#       Format:
#           "<vesc_id> <command_name> <value> <value_type>"
#       Example:
#           "71 CAN_PACKET_SET_POS 180.0 float"
#       Notes:
#           Eight commands are sent together:
#               FL steering
#               FR steering
#               BL steering
#               BR steering
#               FL drive
#               FR drive
#               BL drive
#               BR drive
#           Multiple commands are newline-separated in one message.
#
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from wr_control_msgs.msg import CanReceiveSwerve

COUNTER_MAX = 20

# =============================================================================
# Swerve inverse kinematics
# =============================================================================


def get_wheel_vectors(vehicle_translation, rotational_velocity):
    """
    Calculate the velocity vector for each swerve wheel.

    vehicle_translation:
        [x, y] vehicle translation components.

    rotational_velocity:
        [left_trigger, right_trigger]
    """

    rotational_velocity = -((float(rotational_velocity[0]) + 1) / 2.0) + (
        (float(rotational_velocity[1]) + 1.0) / 2.0
    )

    BODY_HEIGHT = 0.93
    BODY_WIDTH = 0.60

    A = vehicle_translation[0] - (rotational_velocity * (BODY_HEIGHT / 2))
    B = vehicle_translation[0] + (rotational_velocity * (BODY_HEIGHT / 2))

    C = vehicle_translation[1] - (rotational_velocity * (BODY_WIDTH / 2))
    D = vehicle_translation[1] + (rotational_velocity * (BODY_WIDTH / 2))

    FL_vector = [B, D]
    FR_vector = [B, C]
    BL_vector = [A, D]
    BR_vector = [A, C]

    return [FL_vector, FR_vector, BL_vector, BR_vector]


def get_wheel_speed(wheel_vector):
    return math.sqrt(
        wheel_vector[0] * wheel_vector[0] + wheel_vector[1] * wheel_vector[1]
    )


def get_wheel_speeds(wheel_vectors):
    return [
        get_wheel_speed(wheel_vectors[0]),
        get_wheel_speed(wheel_vectors[1]),
        get_wheel_speed(wheel_vectors[2]),
        get_wheel_speed(wheel_vectors[3]),
    ]


def get_wheel_angle(wheel_vector):
    return math.atan2(wheel_vector[0], wheel_vector[1]) * (180 / math.pi)


def get_wheel_angles(wheel_vectors):
    return [
        get_wheel_angle(wheel_vectors[0]),
        get_wheel_angle(wheel_vectors[1]),
        get_wheel_angle(wheel_vectors[2]),
        get_wheel_angle(wheel_vectors[3]),
    ]


# =============================================================================
# Swerve control node
# =============================================================================


class SwerveControl(Node):
    def __init__(self):
        super().__init__("swerve_control")

        # ---------------------------------------------------------------------
        # VESC IDs
        #
        # First ID = drive motor
        # Second ID = steering motor
        # ---------------------------------------------------------------------

        self.vesc_ids = {
            "FL": ["70", "71"],
            "FR": ["72", "73"],
            "BL": ["74", "75"],
            "BR": ["76", "77"],
        }

        # ---------------------------------------------------------------------
        # Swerve control parameters
        # ---------------------------------------------------------------------

        self.max_rpm = 14000
        self.max_rpm_change = 180
        self.max_rpm_drop = 1000
        self.acceleration_ceil = 2000.0

        self.limit_rotation = -10
        self.wheels_straight_angle = 180
        self.angle_error_threshold = 1.5
        self.kI = 100

        # ---------------------------------------------------------------------
        # Rover motion input
        # ---------------------------------------------------------------------

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "swerve",
            self.swerve_callback,
            10,
        )

        # ---------------------------------------------------------------------
        # Steering encoder feedback
        #
        # A single CanReceiveSwerve message carries all four absolute encoder positions
        # ---------------------------------------------------------------------

        self.subscription_encoders = self.create_subscription(
            CanReceiveSwerve,
            "can_receive_swerve",
            self.encoder_callback,
            10,
        )

        # ---------------------------------------------------------------------
        # CAN command publisher
        #
        # This should match the topic consumed by wr_can
        # ---------------------------------------------------------------------

        self.publisher_ = self.create_publisher(
            String,
            "can_send",
            10,
        )

        # ---------------------------------------------------------------------
        # Publish CAN commands at 20 Hz
        # ---------------------------------------------------------------------

        self.publisher_timer = self.create_timer(
            0.05,
            self.publish,
        )

        # ---------------------------------------------------------------------
        # Encoder correction state
        # ---------------------------------------------------------------------

        # collected_* means "a real encoder reading has arrived for this wheel"
        # current_enc_* is the latest steering angle in the VESC convention (180 = straight)
        # TODO: verify that when wheels are straight the VESCs report 180

        self.collected_FL = False
        self.collected_FR = False
        self.collected_BL = False
        self.collected_BR = False

        self.current_enc_FL = 0.0
        self.current_enc_FR = 0.0
        self.current_enc_BL = 0.0
        self.current_enc_BR = 0.0

        # ---------------------------------------------------------------------
        # Motor control state
        # ---------------------------------------------------------------------

        self.error_FL = 0.0
        self.error_FR = 0.0
        self.error_BL = 0.0
        self.error_BR = 0.0

        self.counter_FL = 0
        self.counter_FR = 0
        self.counter_BL = 0
        self.counter_BR = 0

        self.prev_rpm_FL = 0.0
        self.prev_rpm_FR = 0.0
        self.prev_rpm_BL = 0.0
        self.prev_rpm_BR = 0.0

        # ---------------------------------------------------------------------
        # Latest CAN commands
        # ---------------------------------------------------------------------

        self.can_msg_rpm_FL = String()
        self.can_msg_rpm_FR = String()
        self.can_msg_rpm_BL = String()
        self.can_msg_rpm_BR = String()

        self.can_msg_angle_FL = String()
        self.can_msg_angle_FR = String()
        self.can_msg_angle_BL = String()
        self.can_msg_angle_BR = String()

        self.can_msg_rpm_FL.data = (
            self.vesc_ids["FL"][0] + " CAN_PACKET_SET_DUTY 0.0 float"
        )

        self.can_msg_rpm_FR.data = (
            self.vesc_ids["FR"][0] + " CAN_PACKET_SET_DUTY 0.0 float"
        )

        self.can_msg_rpm_BL.data = (
            self.vesc_ids["BL"][0] + " CAN_PACKET_SET_DUTY 0.0 float"
        )

        self.can_msg_rpm_BR.data = (
            self.vesc_ids["BR"][0] + " CAN_PACKET_SET_DUTY 0.0 float"
        )

        self.can_msg_angle_FL.data = (
            self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS 180.0 float"
        )

        self.can_msg_angle_FR.data = (
            self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS 180.0 float"
        )

        self.can_msg_angle_BL.data = (
            self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS 180.0 float"
        )

        self.can_msg_angle_BR.data = (
            self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS 180.0 float"
        )

        self.get_logger().info("Started swerve node")

    # =========================================================================
    # Encoder callback
    # =========================================================================

    def encoder_callback(self, msg):
        """
        Handle absolute steering encoder positions for all four wheels.

        Readings are already in the VESC convention (180 = straight), the
        same frame used by CAN_PACKET_SET_POS, so they are used as-is.
        No startup calibration is performed.
        """

        readings = {
            "FL": msg.fl_encoder,
            "FR": msg.fr_encoder,
            "BL": msg.bl_encoder,
            "BR": msg.br_encoder,
        }

        for wheel, value in readings.items():
            setattr(self, f"collected_{wheel}", True)
            setattr(self, f"current_enc_{wheel}", value % 360)

    # =========================================================================
    # Swerve input callback
    # =========================================================================

    def swerve_callback(self, msg):

        motion = msg.data

        self.get_logger().info(
            f"/swerve received: forward={motion[0]:.2f} strafe={motion[1]:.2f} "
            f"left_trigger={motion[2]:.2f} right_trigger={motion[3]:.2f}"
        )

        # Axis mapping:
        #   motion[0] = forward/back
        #   motion[1] = strafe
        #   motion[2] = left trigger
        #   motion[3] = right trigger
        #
        # The IK deliberately swaps motion[0] and motion[1]

        wheel_vectors = get_wheel_vectors(
            [motion[1], motion[0]],
            [motion[2], motion[3]],
        )

        wheel_speeds = get_wheel_speeds(wheel_vectors)
        wheel_angles = get_wheel_angles(wheel_vectors)

        # Keep steering modules within +/-90 degrees by reversing
        # wheel direction when necessary

        for i in range(4):
            if wheel_angles[i] < -90.0:
                wheel_angles[i] += 180.0
                wheel_speeds[i] *= -1.0

            elif wheel_angles[i] >= 90.0:
                wheel_angles[i] -= 180.0
                wheel_speeds[i] *= -1.0

        # ---------------------------------------------------------------------
        # Process each wheel
        # ---------------------------------------------------------------------

        self.process_wheel(
            "FL",
            wheel_speeds[0],
            wheel_angles[0],
        )

        self.process_wheel(
            "FR",
            wheel_speeds[1],
            wheel_angles[1],
        )

        self.process_wheel(
            "BL",
            wheel_speeds[2],
            wheel_angles[2],
        )

        self.process_wheel(
            "BR",
            wheel_speeds[3],
            wheel_angles[3],
        )

    # =========================================================================
    # Wheel processing
    # =========================================================================

    def process_wheel(self, wheel, speed, angle):

        # -------------------------------------------------------------
        # Convert IK wheel angle (degrees, 0 = straight, +/-90) into the
        # steering VESC position: 4:1 reduction, centered on 180
        # -------------------------------------------------------------

        angle = angle / 4 + 180

        # -------------------------------------------------------------
        # Encoder correction
        # -------------------------------------------------------------

        collected = getattr(self, f"collected_{wheel}")
        counter = getattr(self, f"counter_{wheel}")
        current_enc = getattr(self, f"current_enc_{wheel}")
        error = getattr(self, f"error_{wheel}")

        if collected and angle == self.wheels_straight_angle:
            counter += 1

            if counter > COUNTER_MAX:
                if (
                    abs(self.wheels_straight_angle - current_enc)
                    > self.angle_error_threshold
                ):
                    error -= (self.wheels_straight_angle - current_enc) / self.kI

                    error = math.copysign(
                        min(abs(error), 10),
                        error,
                    )
        else:
            counter = 0

        angle += error

        setattr(self, f"counter_{wheel}", counter)
        setattr(self, f"error_{wheel}", error)

        # -------------------------------------------------------------
        # Steering angle safety limit
        # -------------------------------------------------------------

        if angle < 135 + self.limit_rotation or angle > 225 - self.limit_rotation:
            self.get_logger().error(
                f"SENT INCORRECT ANGLE OF {angle}. Has to be between 135-225"
            )
        else:
            steering_id = self.vesc_ids[wheel][1]

            msg = getattr(self, f"can_msg_angle_{wheel}")

            msg.data = steering_id + " CAN_PACKET_SET_POS " + str(angle) + " float"

        # -------------------------------------------------------------
        # Drive speed / acceleration limiting
        # -------------------------------------------------------------

        rpm = speed * self.max_rpm

        prev_rpm = getattr(
            self,
            f"prev_rpm_{wheel}",
        )

        delta = rpm - prev_rpm

        is_accelerating = abs(rpm) > abs(prev_rpm)

        if is_accelerating:
            if (
                abs(delta) > self.max_rpm_change
                and abs(prev_rpm) <= self.acceleration_ceil
            ):
                delta = math.copysign(
                    self.max_rpm_change,
                    delta,
                )
        else:
            if abs(delta) > self.max_rpm_drop:
                delta = math.copysign(
                    self.max_rpm_drop,
                    delta,
                )

        rpm = prev_rpm + delta

        setattr(
            self,
            f"prev_rpm_{wheel}",
            rpm,
        )

        drive_id = self.vesc_ids[wheel][0]

        msg = getattr(
            self,
            f"can_msg_rpm_{wheel}",
        )

        msg.data = (
            drive_id + " CAN_PACKET_SET_DUTY " + str(rpm / self.max_rpm) + " float"
        )

    # =========================================================================
    # CAN publisher
    # =========================================================================

    def publish(self):

        combined_msg = "\n".join(
            [
                self.can_msg_angle_BR.data,
                self.can_msg_angle_FL.data,
                self.can_msg_angle_FR.data,
                self.can_msg_angle_BL.data,
                self.can_msg_rpm_FL.data,
                self.can_msg_rpm_FR.data,
                self.can_msg_rpm_BL.data,
                self.can_msg_rpm_BR.data,
            ]
        )

        msg = String()
        msg.data = combined_msg

        self.publisher_.publish(msg)


# =============================================================================
# Main
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    node = SwerveControl()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
