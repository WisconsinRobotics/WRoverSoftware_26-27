This node publishes the yaw measured from the Pigeon2 IMU's gyroscope to /imu at 100 Hz.

Usage example:

1. Build and source in the workspace:
```bash
colcon build
source install/setup.bash
```

2. Start the imu node:
```bash
ros2 run wr_imu imu
```

3. In another terminal, source the workspace and echo the IMU data:
```bash
source install/setup.bash
ros2 topic echo /imu
```

The output should look like:
```
data: 12.345
---
data: 12.367
---
data: 12.389
---
```
