# FROM dustynv/ros:humble-pytorch-l4t-r35.4.1
FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

# NOVNC remote desktop Nabbed from https://github.com/theasp/docker-novnc/blob/master/Dockerfile

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update && \
    apt-get install -y \
      bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb && \
    rm -rf /var/lib/apt/lists/

# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes

# DISPLAY_WIDTH=1600 \
# DISPLAY_HEIGHT=900 \
    
COPY novnc /novnc
# CMD ["/novnc/entrypoint.sh"]
EXPOSE 8080


#@  You can create an x11vnc password file by running:       @#
#@                                                           @#
#@       x11vnc -storepasswd password /path/to/passfile      @#
#@  or   x11vnc -storepasswd /path/to/passfile               @#
#@  or   x11vnc -storepasswd                                 @#
#@                                                           @#
#@  (the last one will use ~/.vnc/passwd)                    @#
#@                                                           @#
#@  and then starting x11vnc via:                            @#
#@                                                           @#
#@      x11vnc -rfbauth /path/to/passfile                    @#




# ----------------------------------------------------------------------------

ENV ROS2_WS=/ros_ws \
    ROS_PACKAGE=deepdrive \
    DISPLAY_WIDTH=1600 \
    DISPLAY_HEIGHT=900

WORKDIR $ROS_ROOT
# ADD *.repos ./
ADD install_deps.sh ./
RUN bash install_deps.sh



# source /ros_ws/install/setup.bash

# Install deps
# RUN rosdep update --rosdistro=$ROS_DISTRO
# RUN sudo apt-get update
# RUN rosdep install --from-paths src --ignore-src -r -y

# Build
# RUN . /opt/ros/${ROS_DISTRO}/install/setup.bash && colcon build --symlink-install --merge-install

# apt-get upgrade -y && \

# RUN apt-get update && \
    # apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers && \
    # rm -rf /var/lib/apt/lists/

# RUN git clone https://github.com/ros-controls/ros2_control_demos.git -b ${ROS_DISTRO} src/ros2_control_demos && \
    # bash -c "colcon build --merge-install --symlink-install --packages-ignore gmock_vendor"

# WORKDIR $ROS_ROOT/src/$ROS_PACKAGE/

# ROS2_CONTROL

# RUN mkdir $ROS_ROOT/src

# Build project

# CMD ros2 launch ros2_control_demo_example_1 rrbot.launch.py start_rviz:=false

# COPY . $ROS_ROOT/src/deepdrive/

# RUN rosdep install -i --from-path src --ignore-src -r -y --rosdistro $ROS_DISTRO
# RUN colcon build --merge-install --symlink-install

# colcon build --merge-install --symlink-install