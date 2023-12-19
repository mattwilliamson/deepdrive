#!/usr/bin/env bash

set -e

mkdir -p $ROS_ROOT/src
cd $ROS_ROOT

source /opt/ros/${ROS_DISTRO}/install/setup.bash

apt-get update && rosdep update

# skip installation of some conflicting packages
SKIP_KEYS="libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"
SKIP_KEYS="$SKIP_KEYS rti-connext-dds-6.0.1 ignition-cmake2 ignition-math6"

rosinstall_generator --deps --rosdistro ${ROS_DISTRO} $@ > ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall
cat ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall
vcs import src < ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall
apt-get update && rosdep update
rosdep install -i --from-path ./ --ignore-src -r -y --rosdistro $ROS_DISTRO --skip-keys "$SKIP_KEYS"
colcon build --symlink-install --merge-install

rm -rf /var/lib/apt/lists/

echo echo SOURCING ROS /ros_ws/install/setup.bash >> ~/.bashrc
echo source /ros_ws/install/setup.bash >> ~/.bashrc

mkdir -p $ROS2_WS/src
cd $ROS2_WS
colcon build --symlink-install
