#!/bin/bash

# Uses can0 by default
interface=can0

# Restart the CAN interface at 1Mbps and set queue length to 1000 packets
sudo ip link set $interface down
sudo ip link set $interface type can bitrate 1000000
sudo ip link set $interface up
sudo ip link set $interface txqueuelen 1000

# Launch the ROS2 node
source install/setup.bash
ros2 launch wr_can can_launch.py
