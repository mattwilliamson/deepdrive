imu_bno08x
```sh
colcon build --symlink-install --packages-select imu_bno08x
source /ros_ws/install/setup.sh
ros2 run imu_bno08x imu_bno08x_publisher

ros2 launch imu_bno08x imu.launch.py
ros2 launch imu_bno08x display.launch.py
```

TODO: shake detector only lasts for a moment. Set a trigger. Param?
TODO: tf publisher to show imu working in rviz
TODO: covariance?
TODO: shake detection
TODO: change IMU axes ENU etc