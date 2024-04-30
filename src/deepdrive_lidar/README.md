# LD19 LIDAR Scanner

## Running

```sh
ros2 launch deepdrive_lidar ldlidar_with_mgr.launch.py
```


## Visualization (Mac)
```sh
mamba activate ros_env
colcon build --symlink-install

rviz2 -d ~/Dropbox/Robotics/ldrobot-lidar-ros2/ldlidar_node/config/ldlidar.rviz
```

## Visualization (Linux)
```sh
ros2 launch ldlidar_node ldlidar_rviz2.launch.py
```



## Installation
### On Host Machine

```sh
sudo apt install libudev-dev
cd src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh

screen -L /dev/ldlidar 230400
```

