# Lidar Speed Test

Figure out max speed at max effort for odometry reference

```sh
make dockershell
cd src/deepdrive_hardware # so we can keep it around

# Record the scan topic for playback
ros2 bag record scan
ros2 launch deepdrive_bringup lidar.launch.xml

ros2 launch deepdrive_bringup speed_test.launch.xml use_lidar:=false
ros2 bag play rosbag2_2024_01_27-10_53_22
```

```sh
ros2 launch deepdrive_bringup launch_control.launch.py
ros2 run deepdrive_teleop teleop_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```