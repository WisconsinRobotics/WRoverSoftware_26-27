from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ublox_dgnss_node",
                executable="ublox_dgnss_node",
                name="gnss",
                output="screen",
                parameters=[
                    {
                        "DEVICE_FAMILY": "F9P",
                        "CFG_MSGOUT_UBX_NAV_PVT_USB": 1,
                    }
                ],
            ),
        ]
    )
