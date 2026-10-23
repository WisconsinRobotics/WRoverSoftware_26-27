The launch file runs the ublox_dgnss_node, a third-party ROS 2 driver for u-blox GNSS receivers.
It publishes GNSS information to /ubx_nav_pvt, including the latitude and longitude.

Usage example:

1. Connect the u-blox GNSS receiver and run:
```bash
ls /dev/tty*
```

You should see something like:
```bash
/dev/ttyACM0
```

2. Build and source in the workspace:
```bash
colcon build
source install/setup.bash
```

3. Launch the gnss node:
```bash
ros2 launch wr_gnss gnss_launch.py
```

4. In another terminal, source the workspace and echo /ubx_nav_pvt:
```bash
source install/setup.bash
ros2 topic echo /ubx_nav_pvt
```

The output should look like:
```
header:
  stamp:
    sec: 1728061234
    nanosec: 123456789
  frame_id: "gps"
i_tow: 345678900
year: 2026
month: 10
day: 5
hour: 19
min: 50
sec: 34
valid: 7
t_acc: 50
nano: 123456
fix_type: 3
flags: 3
flags2: 0
num_sv: 18
lon: -894012345
lat: 433821234
height: 285430
height_msl: 276210
h_acc: 1200
v_acc: 1800
vel_n: 15
vel_e: -8
vel_d: 2
g_speed: 17
head_mot: 12345678
s_acc: 250
head_acc: 500000
p_dop: 150
reserved0: 0
reserved1: 0
head_veh: 12345678
mag_dec: 0
mag_acc: 0
```
