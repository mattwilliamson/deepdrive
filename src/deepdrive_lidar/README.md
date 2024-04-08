# LD19 LIDAR Scanner

## Running

```sh
ros2 launch deepdrive_lidar ldlidar_with_mgr.launch.py
```


## Visualization (Mac)
```sh
mamba activate ros_env

rviz2 -d ~/Dropbox/Robotics/ldrobot-lidar-ros2/ldlidar_node/config/ldlidar.rviz
```

## Visualization (Linux)
```sh
ros2 launch ldlidar_node ldlidar_rviz2.launch.py
```



## Installation
### On Host Machine

**FOR NOW JUST USE DEFAULT MAPPING AND SKIP THE FOLLOWING WHICH MAPS IT TO A STATIC PATH UNDER /dev/**



```sh
sudo apt install libudev-dev
./create_udev_rules.sh
```

