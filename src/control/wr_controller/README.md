This node should run on the basestation, after connecting to Xbox controllers.
This node reads Xbox controller inputs and publishes a swerve message ([forward, swerve, left_trigger, right_trigger]) to the "/swerve" topic.

Usage example:

1. Connect Xbox controllers to the basestation.

2. On the basestation, build and source in the workspace:
```bash
colcon build
source install/setup.bash
```

1. Start the drive controller node:
```bash
ros2 run wr_controller drive_controller
```

2. Start the swerve node in a new terminal:
```bash
ros2 run wr_drive swerve
```

This should print in the new terminal:
```
[INFO] [1790624958.925787639] [swerve_control]: /swerve received: forward=0.00 strafe=0.00 left_trigger=1.00 right_trigger=1.00
```
