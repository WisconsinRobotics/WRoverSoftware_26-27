Publishing a swerve message ([forward, swerve, left_trigger, right_trigger]) to the "/swerve" topic will drive the motors accordingly using inverse kinematics.
The swerve node also uses the absolute encoder position feedback from "/can_swerve_receive" to adjust the motors.

Usage example:

1. Build and source in the workspace:
```bash
colcon build
source install/setup.bash
```

2. Start the swerve node:
```bash
ros2 run wr_drive swerve
```

3. In another terminal, publish an example:
```bash
ros2 topic pub /swerve std_msgs/msg/Float32MultiArray "data: [0.0, 0.0, 1.0, 1.0]"
```

This should print in the new terminal:
```
publishing #1: std_msgs.msg.Float32MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.0, 0.0, 1.0, 1.0])
```

And in the previous terminal:
```
[INFO] [1790624958.925787639] [swerve_control]: /swerve received: forward=0.00 strafe=0.00 left_trigger=1.00 right_trigger=1.00
```
