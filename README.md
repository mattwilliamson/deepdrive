# DeepDrive


## TODO

- [ ] Move jetson again
- [ ] Isaac ROS Simulator?
- [ ] Use isaac ros vslam
- [ ] gasket for lidar
- [ ] move bno08x to separate repo
- [ ] matek optical flow module
- [ ] wide angle camera nodelet to take snapshots? https://github.com/ros-drivers/gscam/blob/master/examples/nodelet_pipeline.launch
- [ ] gazebo range simulation
- [ ] /camera_depth.camera.i_restart_on_diagnostics_error
- [ ] /camera_depth.rgb.i_enable_preview
- [x] lidar odom? https://github.com/PRBonn/kiss-icp
- [ ] m-explore
- [ ] visual odom: https://github.com/MIT-SPARK/Kimera-VIO
- [ ] ROS ISAAC VSLAM - https://github.com/luxonis/depthai-ros/commit/038f107800ef40ea1a3359bc37f13614fbd5b572
- [ ] Run ROS ISAAC in separate container because they have all the docker images and such
- [x] Battery state publisher from pico
- [x] current/volt sensor
- [ ] twist-mux 
- [ ] calibrate imu covariance
- [ ] find_object_2d
- [ ] how to annotate rooms?
- [ ] behavior trees? https://py-trees-ros.readthedocs.io/en/devel/about.html https://github.com/BehaviorTree/BehaviorTree.CPP
- [x] BNO080 publisher - switch to c++ https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
- [ ] rclcpp::NodeOptions().use_intra_process_comms(true)
- [ ] set collision detection on IMU
- [ ] stall detection based on rotary encoders
- [ ] lcd display?
- [ ] m-explore while streaming images to room inference. remember where the photos were taken
- [ ] navigation2
- [ ] slam 2d/3d slam-toolbox/rtabmap
- [ ] param/deepdrive.yaml ros params for wheelbase, serial port, etc
- [ ] oak d lite camera simulated
- [ ] Add depthai and cache models
- [ ] gps


## Setup

### Deepdrive motor controller

Follow instructions at:
https://github.com/mattwilliamson/deepdrive_motor_controller
and flash your motor controllers.

### Deepdrive Docker

```sh
git clone https://github.com/mattwilliamson/deepdrive.git
cd deepdrive
git submodule init
git submodule update
make dockershell

colcon build...
```


# Simulator
Run on x86_64 linux machine

```sh
make dockersimshell

ros2 launch deepdrive_simulations deepdrive_house.launch.py
ros2 run deepdrive_teleop teleop_keyboard --ros-args -r /cmd_vel:=/deepdrive_micro/cmd_vel
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 launch deepdrive_bringup foxglove_bridge_launch.xml

ros2 launch deepdrive_description display.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard




export DEEPDRIVE_MODEL=deepdrive_deepdrive
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/install/deepdrive_simulations/share/deepdrive_simulations/models
ros2 launch deepdrive_nav2_bringup dd_simulation_launch.py headless:=False

export DEEPDRIVE_MODEL=deepdrive
export LDS_MODEL='LDS-01'
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/install/deepdrive_simulations/share/deepdrive_simulations/models
ros2 launch deepdrive_simulations deepdrive_simulation_launch.py headless:=False robot_name:=deepdrive_deepdrive 
# slam:=True


ros2 launch deepdrive_navigation2 navigation2.launch.py

## Launch Gazebo and RVIZ
ros2 launch deepdrive_simulations deepdrive_dqn_stage1.launch.py
ros2 launch deepdrive_bringup rviz2.launch.py

# Camera
ros2 launch depthai_examples stereo_inertial_node.launch.py camera_model:=OAK-D-LITE enableRviz:=False depth_aligned:=True rectify:=True

ros2 launch depthai_ros_driver rgbd_pcl.launch.py camera_model:=OAK-D-LITE enableRviz:=False depth_aligned:=True rectify:=True r_set_man_focus:=True r_focus:=1 imuMode:=1
# r_focus:=1

# Foxglove Studio Bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml




DEEPDRIVE_MODEL

# TODO: find files waffle.*

```

## Robot

Latest testing working:
```sh
ros2 launch deepdrive_bringup robot.launch.py
ros2 run deepdrive_teleop teleop_keyboard
```

```sh
# ros2 run deepdrive_hardware motor_controller

ros2 launch deepdrive_bringup robot.launch.py
ros2 run deepdrive_teleop teleop_keyboard
```

## LIDAR
```sh
ros2 launch deepdrive_lidar ldlidar_with_mgr.launch.py
```

## Working Test
```sh
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_sim_time:=True
```

```sh
ros2 launch deepdrive_nav2_bringup tb3_simulation_launch.py headless:=False use_sim_time:=True
```


### Vizualize URDF
```sh
xacro src/deepdrive_description/urdf/deepdrive_deepdrive.xacro > src/deepdrive_description/urdf/deepdrive_deepdrive.urdf
urdf-viz src/deepdrive_description/urdf/deepdrive_deepdrive.urdf
```

---

https://github.com/ros-planning/navigation2_tutorials
https://github.com/ros-planning/navigation2_tutorials/tree/master/sam_bot_description
https://navigation.ros.org/
https://robofoundry.medium.com/modifying-stl-files-to-work-with-urdf-686cb15a4488
https://github.com/ros-mobile-robots/diffbot

## TODO:
- [ ] Move novnc as an optional startup script
- [ ] File structure https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html
- [ ] Host networking doesn't let x11 listen on the port


https://github.com/theasp/docker-novnc
https://github.com/AtsushiSaito/docker-ubuntu-sweb
https://github.com/Tiryoh/docker-ros2-desktop-vnc


## Build Base Jetson image first 
with Ubuntu 22.04 (Jammy) instead of Focal

List packages:
```
~/src/jetson-containers$ ./build.sh --list-packages

Namespace(base='', build_flags='', list_packages=True, logs='', multiple=False, name='', no_github_api=False, package_dirs=[''], packages=[], push='', show_packages=False, simulate=False, skip_errors=False, skip_packages=[''], skip_tests=[''], test_only=[''], verbose=False)

-- L4T_VERSION=35.4.1
-- JETPACK_VERSION=5.1.2
-- CUDA_VERSION=11.4.315
-- LSB_RELEASE=20.04 (focal)
```


```sh
docker login nvcr.io

cd ~/Dropbox/Robotics/jetson-containers
BASE_IMAGE
dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

./build.sh ros
ros-humble-desktop

Packages:
./build.sh --base l4t-jetpack ros:foxy-desktop
# ./build.sh --base l4t-jetpack ros:humble-desktop l4t-pytorch jupyterlab

langchain llamaspeak llava nanodb numpy
```

```sh
sudo apt install docker-compose docker-compose-plugin
git clone https://github.com/theasp/docker-novnc.git
cd docker-novnc
docker build -t docker-novnc .

```

```sh
cd ~/src/deepdrive
docker build -t deepdrive .
docker run -it --rm -p 8080:8080 -e DISPLAY_WIDTH=1600 -e DISPLAY_HEIGHT=900 deepdrive /novnc/entrypoint.sh


```


```sh
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# ros2 launch deepdrive_description display.launch.py
# ros2 launch deepdrive_bringup rviz2.launch.py
# ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

# Navigation

https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

```sh
ros2 launch deepdrive_bringup robot.launch.py

ros2 launch deepdrive_bringup foxglove_bridge_launch.xml

# Start Nav2
ros2 launch deepdrive_nav2_bringup navigation_launch.py use_sim_time:=False params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml

# Start slam_toolbox
ros2 launch deepdrive_nav2_bringup slam_launch.py use_sim_time:=False params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml

# Start teleoperation
ros2 run deepdrive_teleop teleop_keyboard
# --ros-args -r /cmd_vel:=/deepdrive_micro/cmd_vel

ros2 bag record -a -o src/bags/slam-2024-05-30
ros2 bag play src/bags/slam-2024-04-30

```

Save map
```sh
ros2 run nav2_map_server map_saver_cli -f $ROS2_WS/src/deepdrive_navigation2/map/desk
ros2 run nav2_map_server map_saver_cli -f $ROS2_WS/src/deepdrive_navigation2/map/downstairs
```

### If you have a map already

```sh
ros2 launch deepdrive_bringup robot.launch.py

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml map:=$ROS2_WS/src/deepdrive_navigation2/map/desk.yaml

ros2 launch deepdrive_nav2_bringup bringup_launch.py slam:=False use_sim_time:=False autostart:=True map:=$ROS2_WS/src/deepdrive_navigation2/map/desk.yaml params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml

#ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=$ROS2_WS/src/deepdrive_navigation2/map/desk.yaml params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml
```

## Visualize on a mac (Robostack)
```sh
mamba activate ros_env
mamba install ros-humble-nav2-rviz-plugins
mamba install ros-humble-rviz2

colcon build --symlink-install --packages-select=deepdrive_description

rviz2 -d src/deepdrive_nav2_bringup/rviz/nav2_default_view.rviz
```


# Electronics

## Lidar

LD19

check [./src/deepdrive_lidar/README.md](./src/deepdrive_lidar/README.md)


---

# IMU BNO080
**currently deprecated in favor of ICM20948 on deepdrive_micro board
`sudo /opt/nvidia/jetson-io/jetson-io.py`
```sh
sudo apt-get install libi2c-dev
sudo i2cdetect -r -y 1
```

pin3 - i2c2
pin5 - i2c2


7 pins up from bottom


