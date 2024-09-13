# deepdrive_control

TODO: low-latency realtime https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html

ros2_control diff_drive_controller to interface with [deepdrive_motor_controller](https://github.com/mattwilliamson/deepdrive_motor_controller)

Uses `deepdrive_control/DeepdriveSystemHardware` as the hardware interface. 

```sh
colcon build --symlink-install --packages-select deepdrive_control

source install/setup.sh
ros2 launch deepdrive_control deepdrive_control.launch.py

```

```sh
ros2 launch deepdrive_bringup robot.launch.py
```

