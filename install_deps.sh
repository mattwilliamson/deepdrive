#!/usr/bin/env bash

set -e

echo "################################"
echo "-> Installing dependencies for '$@'"

mkdir -p $ROS_ROOT/src
cd $ROS_ROOT

source /opt/ros/${ROS_DISTRO}/install/setup.bash

apt-get update && rosdep update

# skip installation of some conflicting packages
SKIP_KEYS="libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"
SKIP_KEYS="$SKIP_KEYS rti-connext-dds-6.0.1 ignition-cmake2 ignition-math6"

echo "-> Generating rosintall file..."
rosinstall_generator --deps --rosdistro ${ROS_DISTRO} $@ > ros2.${ROS_DISTRO}.rosinstall
cat ros2.${ROS_DISTRO}.rosinstall

echo "-> Cloning dependency sources..."
vcs import src < ros2.${ROS_DISTRO}.rosinstall

echo "-> Updating rosdep..."
apt-get update && rosdep update

# This fails due to no license, so let's just remove it
rm -rf /opt/ros/humble/build/pluginlib/pluginlib_enable_plugin_testing

echo "-> Installing rosdep dependencies..."
rosdep install -i --from-path ./ --ignore-src -r -y --rosdistro $ROS_DISTRO --skip-keys "$SKIP_KEYS"

echo "-> Running colcon build..."
colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cmake-args -DBUILD_TESTING=OFF \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    --cmake-args -DBUILD_SHARED_LIBS=ON \
    --cmake-args -DHUNTER_KEEP_PACKAGE_SOURCES=ON

rm -rf /var/lib/apt/lists/

cd $ROS2_WS
colcon build --symlink-install
