# deepdrive_description

This has the URDF files and such to describe how to robot looks and behaves physically.

```sh
make dockersimshell
ros2 launch deepdrive_description display.launch.py
ros2 run deepdrive_teleop teleop_keyboard use_sim_time:=true --ros-args --remap /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

```sh
urdf-viz src/deepdrive_description/urdf/deepdrive_deepdrive.xacro
```

## OSX / Robostack

```sh
mamba activate ros_env
colcon build --symlink-install --packages-select deepdrive_description
source install/setup.zsh
ros2 launch deepdrive_description display.launch.py
```

Open up Foxglove Studio