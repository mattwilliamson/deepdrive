# Lidar Speed Test

Figure out max speed at max effort for odometry reference

```sh
make dockershell
cd src/deepdrive_hardware # so we can keep the bag recording around

# Record the scan topic for playback
ros2 bag record scan
ros2 launch deepdrive_bringup lidar.launch.xml

# Drive back and forth to see max speed and acceleration
ros2 launch deepdrive_bringup launch_control.launch.py
ros2 run deepdrive_teleop teleop_keyboard

# Replay the bag to test the script
ros2 launch deepdrive_bringup speed_test.launch.xml use_lidar:=false
ros2 bag play rosbag2_lidar_driving
```
