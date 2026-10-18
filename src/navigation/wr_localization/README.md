This node publishes the pose data generated from the sensors. It sends pose data after every update, only stopping when the GNSS or IMU data gets too old.

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

The pose should look like:
```
x: 43.07
y: -89.4
theta: 1.0
```
