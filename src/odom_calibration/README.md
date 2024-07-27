# odom_calibration

Simple node to check different odom ources for outliers.

```sh
ros2 launch deepdrive_bringup robot.launch.py

colcon build --symlink-install --packages-select=odom_calibration
source install/setup.sh
ros2 launch odom_calibration calibrate.launch.py


```