# =============================================================================
# Background:
#   The CAN (Controller Area Network) bus allows communication between microcontrollers.
#   The Jetson Nano uses the CAN bus to communicate with VESCs (Vedder Electronic Speed Controller).
#   ROS 2 nodes send command frames over the bus to set motor duty, RPM, current, or position.
#   VESCs receive these frames to control motors, they also broadcast status frames over the CAN bus.
#   VESCs broadcast absolute encoder position in the STATUS_4 frame (VESC Command ID 16).
#   VESCs broadcast motor current and RPM in the general STATUS frame (VESC Command ID 9).
#
# =============================================================================
# Brief:
#   This ROS 2 node converts command strings into 29-bit extended CAN bus frames
#   with 4-byte big-endian payload data.
#   It also decodes the status frames of swerve and arm motors.
#
# =============================================================================
# Subscribes to:
#   '/can_send' (std_msgs/String)
#       Description:
#           VESC command string to be sent over the CAN bus.
#       Format:
#           "<vesc_id> <command_name> <value> <value_type>"
#       Example:
#           # Set motor with VESC ID 1 to 3000 RPM (int)
#           "1 CAN_PACKET_SET_RPM 3000 int"
#       Notes:
#           Multiple commands may be newline-separated in a single message,
#           wr_drive sends all eight wheel commands as one 8-line string.
#
# =============================================================================
# Publishes to:
#   '/can_receive_swerve' (CanReceiveSwerve)
#       Description:
#           Absolute encoder positions of swerve motors.
#       Format:
#           fl_encoder (float32)
#           fr_encoder (float32)
#           bl_encoder (float32)
#           br_encoder (float32)
#       Example:
#           # Driving straight forward
#           fl_encoder: 0.0
#           fr_encoder: 0.0
#           bl_encoder: 0.0
#           br_encoder: 0.0
#
#   '/can_receive_arm' (CanReceiveArm)
#       TODO
#
# =============================================================================

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from wr_control_msgs.msg import CanReceiveSwerve
from wr_control_msgs.msg import CanReceiveArm

import can
import time
import threading

# =============================================================================
# VESC IDs
# =============================================================================

SIDE_TO_SIDE_VESC = 80
IN_OUT_VESC = 82
UP_DOWN_VESC = 81
GRIPPER_VESC = 85

FL_VESC = 70
FR_VESC = 72
BL_VESC = 74
BR_VESC = 76

CHANNEL = "can0"

# =============================================================================
# VESC command lookup table
# =============================================================================

COMMAND_TABLE: dict[str, tuple[int, int, bool]] = {
    # Simple commands
    "CAN_PACKET_SET_DUTY": (0, 100000, False),
    "CAN_PACKET_SET_CURRENT": (1, 1000, False),
    "CAN_PACKET_SET_CURRENT_BRAKE": (2, 1000, False),
    "CAN_PACKET_SET_RPM": (3, 1, False),
    "CAN_PACKET_SET_POS": (4, 1000000, False),
    "CAN_PACKET_SET_CURRENT_REL": (10, 100000, False),
    "CAN_PACKET_SET_CURRENT_BRAKE_REL": (11, 100000, False),
    "CAN_PACKET_SET_CURRENT_HANDBRAKE": (12, 1000, False),
    "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL": (13, 100000, False),
    # Status commands
    "CAN_PACKET_STATUS": (9, 0, True),
    "CAN_PACKET_STATUS_2": (14, 0, True),
    "CAN_PACKET_STATUS_3": (15, 0, True),
    "CAN_PACKET_STATUS_4": (16, 0, True),
    "CAN_PACKET_STATUS_5": (27, 0, True),
    "CAN_PACKET_STATUS_6": (28, 0, True),
}


# =============================================================================
# CAN message construction
# =============================================================================


def build_msg(
    command: str,
    value: int | float,
    vesc_id: int,
    raw: bool = False,
):
    """
    Build a VESC CAN message from a command name, value, and VESC ID.

    Returns:
        (can.Message, is_status)

    Raises:
        ValueError if command is not in COMMAND_TABLE.
    """

    try:
        command_id, scaling, is_status = COMMAND_TABLE[command]

    except KeyError:
        raise ValueError(f"Unknown command: '{command}' (value={value})")

    # Arbitration ID:
    # [13 unused bits][8 command bits][8 VESC ID bits]
    arb_id = (command_id << 8) | (vesc_id & 0xFF)

    # Payload: scaled integer, big-endian, 4 bytes, signed
    int_data = int(value * scaling)

    data = int_data.to_bytes(
        4,
        byteorder="big",
        signed=True,
    )

    if raw:
        id_bits = bin(arb_id)[2:].zfill(29)
        return id_bits, data

    return (
        can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=True,
        ),
        is_status,
    )


# =============================================================================
# CAN node
# =============================================================================


class CANNode(Node):
    def __init__(self):

        super().__init__("can")

        # ----- CAN bus -----

        self._bus = can.Bus(
            channel=CHANNEL,
            interface="socketcan",
            can_filters=[
                # ----- STATUS (command 9) -----
                {
                    "can_id": (9 << 8) | SIDE_TO_SIDE_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (9 << 8) | IN_OUT_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (9 << 8) | UP_DOWN_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (9 << 8) | GRIPPER_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                # ----- STATUS_4 (command 16) -----
                {
                    "can_id": (16 << 8) | FL_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (16 << 8) | FR_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (16 << 8) | BL_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
                {
                    "can_id": (16 << 8) | BR_VESC,
                    "can_mask": 0xFFFF,
                    "extended": True,
                },
            ],
        )

        self.get_logger().info(f"CAN bus opened on {CHANNEL}")

        # ----- ROS -> CAN -----

        self._pending: dict[tuple, can.Message] = {}
        self._pending_lock = threading.Lock()
        self._pending_event = threading.Event()

        self.subscription = self.create_subscription(
            String,
            "can_send",
            self._listener_callback,
            10,
        )

        self._sender_thread = threading.Thread(
            target=self._sender_loop,
            daemon=True,
            name="can_sender",
        )

        self._sender_thread.start()

        # ----- CAN -> ROS -----

        self.receive_swerve_publisher = self.create_publisher(
            CanReceiveSwerve,
            "can_receive_swerve",
            10,
        )

        self.receive_arm_publisher = self.create_publisher(
            CanReceiveArm,
            "can_receive_arm",
            10,
        )

        # ----- Receive swerve state -----

        self.receive_swerve = CanReceiveSwerve()

        self.receive_swerve.fl_encoder = 0.0
        self.receive_swerve.fr_encoder = 0.0
        self.receive_swerve.bl_encoder = 0.0
        self.receive_swerve.br_encoder = 0.0

        # ----- Receive arm state -----

        self.receive_arm = CanReceiveArm()

        # TODO

        # ----- Receive timer -----

        self.timer = self.create_timer(
            0.002,
            self._receive_callback,
        )

    # =========================================================================
    # ROS -> CAN
    # =========================================================================

    def _listener_callback(self, msg: String):
        """
        Parse ROS can_send commands and place the newest command into _pending.

        Format:
            <vesc_id> <command> <value> <value_type>
        """

        try:
            lines = msg.data.strip().splitlines()

            for line in lines:
                self.get_logger().info(f'I heard: "{line}"')

                parts = line.split()

                if len(parts) != 4:
                    raise ValueError(f"Expected 4 fields, got {len(parts)}: '{line}'")

                vesc_id = int(parts[0])
                command = parts[1]
                raw_value = parts[2]
                vtype = parts[3]

                value = _parse_value(
                    raw_value,
                    vtype,
                )

                can_msg, is_status = build_msg(
                    command=command,
                    value=value,
                    vesc_id=vesc_id,
                )

                # Status commands are not sent
                if not is_status:
                    key = (
                        vesc_id,
                        command,
                    )

                    # Last-value-wins behavior
                    with self._pending_lock:
                        self._pending[key] = can_msg

                    self._pending_event.set()

        except Exception as e:
            self.get_logger().error(f"Failed to process message '{msg.data}': {e}")

    # =========================================================================
    # CAN sender thread
    # =========================================================================

    def _sender_loop(self):
        """
        Dedicated thread for CAN transmission.

        ROS callbacks never block on CAN I/O.
        """

        MAX_RETRIES = 5
        RETRY_DELAY = 0.001

        while rclpy.ok():
            self._pending_event.wait(timeout=0.1)

            self._pending_event.clear()

            # Snapshot latest commands
            with self._pending_lock:
                snapshot = list(self._pending.values())

                self._pending.clear()

            # Send snapshot
            for can_msg in snapshot:
                for attempt in range(MAX_RETRIES):
                    try:
                        self._bus.send(can_msg)
                        break

                    except can.CanError as e:
                        if attempt < MAX_RETRIES - 1:
                            time.sleep(RETRY_DELAY)

                        else:
                            self.get_logger().warning(
                                f"Dropping CAN message after {MAX_RETRIES} retries: {e}"
                            )

    # =========================================================================
    # CAN -> ROS
    # =========================================================================

    def _receive_callback(self):
        """
        Drain all currently available CAN messages.

        timeout=0.0 makes this non-blocking.
        """

        while True:
            msg = self._bus.recv(timeout=0.0)

            if msg is None:
                break

            self._process_can_message(msg)

    # =========================================================================
    # CAN message processing
    # =========================================================================

    def _process_can_message(self, msg):

        arb_id = msg.arbitration_id

        command_id = (arb_id >> 8) & 0xFF

        vesc_id = arb_id & 0xFF

        data = msg.data

        # ----- STATUS_4 -----

        if command_id == 16:
            position = (
                int.from_bytes(
                    data[6:8],
                    "big",
                    signed=True,
                )
                / 50
            )

            if vesc_id == FL_VESC:
                self.receive_swerve.fl_encoder = position

            elif vesc_id == FR_VESC:
                self.receive_swerve.fr_encoder = position

            elif vesc_id == BL_VESC:
                self.receive_swerve.bl_encoder = position

            elif vesc_id == BR_VESC:
                self.receive_swerve.br_encoder = position

            else:
                return

            # Publish the complete swerve state
            self.receive_swerve_publisher.publish(self.receive_swerve)

        # ----- STATUS -----

        elif command_id == 9:
            # TODO
            # current = (
            #    int.from_bytes(
            #        data[4:6],
            #        "big",
            #        signed=True,
            #    )
            #    / 10
            # )

            # Publish the complete arm state
            self.receive_arm_publisher.publish(self.receive_arm)

    # =========================================================================
    # Cleanup
    # =========================================================================

    def destroy_node(self):

        self.get_logger().info("Shutting down CAN node...")

        self._bus.shutdown()

        super().destroy_node()


# =============================================================================
# Helpers
# =============================================================================


def _parse_value(
    raw: str,
    vtype: str,
) -> int | float:

    match vtype:
        case "float":
            return float(raw)

        case "int":
            return int(raw)

        case "string":
            return raw

        case _:
            raise TypeError(f"Unsupported value type: '{vtype}'")


# =============================================================================
# Main
# =============================================================================


def main(args=None):

    rclpy.init(args=args)

    node = CANNode()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
