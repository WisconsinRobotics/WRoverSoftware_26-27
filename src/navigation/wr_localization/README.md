This node publishes the pose data generated from the sensors.

Usage example:

1. Build and source in the workspace:
```bash
colcon build
source install/setup.bash
```

2. Start the localization node:
```bash
ros2 run wr_localization localization
```

3. In another terminal, source the workspace and echo the pose data:
```bash
source install/setup.bash
ros2 topic echo /pose
```

The output should look like: TODO: verify this
```
x: 1.0
y: 1.0
theta: 1.0
```
