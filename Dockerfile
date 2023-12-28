# FROM dustynv/ros:humble-pytorch-l4t-r35.4.1
FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

ENV ROS2_WS=/ros_ws \
    ROS_PACKAGE=deepdrive \
    DISPLAY_WIDTH=1600 \
    DISPLAY_HEIGHT=900

WORKDIR $ROS_ROOT
ADD setup_ws.sh ./
RUN bash setup_ws.sh \
        xacro \
	    gazebo_ros \
	    robot_localization \
        gazebo_plugins

# x11-apps \
# apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
# apt-get install mesa-utils
# glxgears

# /root/.Xauthority

WORKDIR $ROS2_WS
COPY src $ROS2_WS/src/deepdrive
RUN bash -c "source install/setup.bash && colcon build --symlink-install"
