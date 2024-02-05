# DeepDrive
Heavily borrowed from https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel

## TODO:
- [ ] calibrate motor pwm speed
- [ ] add wheel encoders to stl
- [ ] m-explore
- [ ] m-explore while streaming images to room inference. remember where the photos were taken
- [ ] navigation2
- [ ] slam 2d/3d slam-toolbox/rtabmap
- [ ] separate workspace for depthai?
- [ ] add led ring
- [ ] cooling holes
- [ ] param/deepdrive.yaml
- [ ] oak d lite camera simulated
- [x] URDF deepdrive_deepdrive.urdf
- [ ] Add depthai and cache models
- [x] add wide angle camera
- [ ] https://github.com/BehaviorTree/BehaviorTree.CPP
- [ ] ros params for wheelbase, serial port, etc
- [ ] second IMU / compass
- [ ] gps
- [ ] diagnostics for motor controller

Bringup:
https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/tb3_simulation_launch.py

# Simulator
Run on x86_64 linux machine

```sh
make dockersimshell

ros2 launch deepdrive_gazebo deepdrive_house.launch.py
ros2 run deepdrive_teleop teleop_keyboard
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ros2 launch deepdrive_description display.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard




export DEEPDRIVE_MODEL=deepdrive_deepdrive
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/install/deepdrive_gazebo/share/deepdrive_gazebo/models
ros2 launch deepdrive_nav2_bringup dd_simulation_launch.py headless:=False

export DEEPDRIVE_MODEL=deepdrive
export LDS_MODEL='LDS-01'
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros_ws/install/deepdrive_gazebo/share/deepdrive_gazebo/models
ros2 launch deepdrive_gazebo deepdrive_simulation_launch.py headless:=False robot_name:=deepdrive_deepdrive 
# slam:=True


ros2 launch deepdrive_navigation2 navigation2.launch.py

## Launch Gazebo and RVIZ
ros2 launch deepdrive_gazebo deepdrive_dqn_stage1.launch.py
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
ros2 launch deepdrive_bringup launch_control.launch.py
ros2 run deepdrive_teleop teleop_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

```sh
ros2 run deepdrive_hardware motor_controller

ros2 launch deepdrive_bringup robot.launch.xml
```

## LIDAR
```sh
sudo chmod a+rw /dev/ttyTHS0
screen -L /dev/ttyTHS0 230400
ros2 run tf2_ros static_transform_publisher 0 0 0 0 3.14159 3.14159 oak-d-base-frame laser
ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py port:=/dev/ttyTHS0
```

## Working Test
```sh
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_sim_time:=True
```

```sh
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch deepdrive_nav2_bringup tb3_simulation_launch.py headless:=False use_sim_time:=True
```


### Vizualize URDF
```sh
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
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 launch deepdrive_description display.launch.py
ros2 launch deepdrive_bringup rviz2.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```




# Electronics

## Lidar
LDS-01
hls_lfcd_lds_driver

Connect 5v, gnd, pin 8 and pin 10 for serial

```sh
sudo apt-get install -y screen
sudo chmod a+rw /dev/ttyTHS0
screen -L /dev/ttyTHS0 230400
```

