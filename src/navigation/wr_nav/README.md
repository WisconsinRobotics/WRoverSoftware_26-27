This node publishes rover motion commands to /swerve at 10 Hz for traversal (GNSS), approach (3D), and astronaut tracking. It starts in STOP mode and can be changed by publishing to /nav_mode.

Usage example:

1. Build and source in the workspace:

```bash
colcon build
source install/setup.bash
```

2. Start the navigation node:

```bash
ros2 run wr_nav nav
```

3. In another terminal, source the workspace, set a GNSS target, and select traversal mode:

```bash
source install/setup.bash
ros2 topic pub --once /target_gnss geometry_msgs/msg/Point "{x: 43.07, y: -89.4, z: 0.0}"
ros2 topic pub --once /nav_mode std_msgs/msg/Int32 "{data: 4}"
```

4. View the motion commands, or STOP navigation from another sourced terminal:

```bash
ros2 topic echo /swerve
```

```bash
ros2 topic pub --once /nav_mode std_msgs/msg/Int32 "{data: 0}"
```
