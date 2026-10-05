Publishing a CAN message to the "/can_msg" topic will send the message over can0.
This node also publishes the absolute encoder positions of the swerve motors to "/can_receive_swerve" using the CanReceiveSwerve custom message type.

Usage example:

1. Build in workspace and start the CAN bus:
```bash
colcon build
scripts/roverStart.sh
```

2. roverStart.sh should already start the can node, but you can also start it manually with:
```bash
ros2 run wr_can can
```

3. In another terminal, publish an example:
```bash
source install/setup.bash
ros2 topic pub can_send std_msgs/String "data: 23 CAN_PACKET_SET_CURRENT 51 int"
```

This should print in the new terminal:
```
publishing #1: std_msgs.msg.String(data='23 CAN_PACKET_SET_CURRENT 51 int')
```

And in the previous terminal:
```
[INFO] [1736897861.362478158] [can_subscriber]: I heard: "23 CAN_PACKET_SET_CURRENT 51 int"
```
