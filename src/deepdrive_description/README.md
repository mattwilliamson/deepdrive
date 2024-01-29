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

# Ros2 Control

```sh

ros2 control load_controller controller_manager
ros2 control list_hardware_interfaces
ros2 run deepdrive_teleop teleop_keyboard

ros2 launch deepdrive_bringup launch_sim.launch.py world:=src/deepdrive_description/worlds/obstacles.world
ros2 run deepdrive_teleop teleop_keyboard use_sim_time:=true --ros-args --remap /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

ros2 run controller_manager spawner diff_drive_controller
ros2 run controller_manager spawner joint_state_broadcaster


```