# deepdrive_node

Takes `cmd_vel` and converts it to velocity commands for each wheel. Publishes `odom` and `joint_state`.

Just this package

```sh
colcon build --symlink-install --packages-select deepdrive_node
source install/setup.sh

ros2 launch deepdrive_node deepdrive_node.launch.py
```

Everything

```sh
colcon build --symlink-install
source install/setup.sh

pip install -r src/imu_bno08x/requirements.txt

ros2 launch deepdrive_bringup robot.launch.py

ros2 run deepdrive_teleop teleop_keyboard
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```