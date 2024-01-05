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

# Install opencv (needed for gazebo_plugins)
# TODO: Move this above all install_deps.sh and consolidate those
# Most of these are already installed, but keep them for posterity
# WORKDIR /usr/local/src
# RUN apt-get update && \
#     apt-get install -y \
#         build-essential cmake git pkg-config libgtk-3-dev \
#         libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
#         libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
#         gfortran openexr libatlas-base-dev python3-dev python3-numpy \
#         libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
#         libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev && \
#     git clone https://github.com/opencv/opencv.git && \
#     git clone https://github.com/opencv/opencv_contrib.git && \
#     cd opencv && mkdir -p build && cd build && \
#     cmake -D CMAKE_BUILD_TYPE=RELEASE \
#         -D CMAKE_INSTALL_PREFIX=/usr/local \
#         -D INSTALL_C_EXAMPLES=ON \
#         -D INSTALL_PYTHON_EXAMPLES=ON \
#         -D OPENCV_GENERATE_PKGCONFIG=ON \
#         -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules \
#         -D BUILD_EXAMPLES=ON .. && \
#     make -j6 && \
#     make install && \
#     rm -rf /var/lib/apt/lists/

# Check it's installed
RUN pkg-config --modversion opencv4
RUN python3 -c "import cv2; print(cv2.__version__)"

# ----------------------------------------------------------------------------

# ZSH and TMUX and such

RUN apt-get update && \
    apt-get install -y \
        vim \
        python3-pip \
        python3-pydantic \
        ruby-dev \
        rviz \
        tmux \
        wget \
        xorg-dev \
        xtl-dev \
        zsh && \
    rm -rf /var/lib/apt/lists/
RUN pip3 install setuptools==58.2.0

# TODO: Wrong architecture
# RUN wget https://github.com/openrr/urdf-viz/releases/download/v0.44.0/urdf-viz-x86_64-unknown-linux-gnu.tar.gz && \
#     tar -xvzf urdf-viz-x86_64-unknown-linux-gnu.tar.gz -C /usr/local/bin/ && \
#     chmod +x /usr/local/bin/urdf-viz && \
#     rm -f urdf-viz-x86_64-unknown-linux-gnu.tar.gz

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

RUN gem install tmuxinator && \
    wget https://raw.githubusercontent.com/tmuxinator/tmuxinator/master/completion/tmuxinator.zsh -O /usr/local/share/zsh/site-functions/_tmuxinator

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.zshrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.zshrc
RUN echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.zshrc

RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.zshrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.zshrc
RUN echo 'alias ssetup="source ./install/local_setup.zsh"' >> /root/.zshrc
RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> /root/.zshrc
RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> /root/.zshrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.zshrc

RUN echo "autoload -U bashcompinit" >> /root/.zshrc
RUN echo "bashcompinit" >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

COPY .session.yml /root/.session.yml
COPY .tmux.conf /root/.tmux.conf

CMD [ "tmuxinator", "start", "-p", "/root/.session.yml" ]


# Actual ROS2 stuff

ENV ROS2_WS=/ros_ws \
    ROS_PACKAGE=deepdrive \
    DISPLAY_WIDTH=1600 \
    DISPLAY_HEIGHT=900 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR $ROS_ROOT
RUN echo echo SOURCING ROS /ros_ws/install/setup.bash >> ~/.bashrc && \
    echo source /ros_ws/install/setup.bash >> ~/.bashrc && \
    mkdir -p $ROS2_WS/src \
    mkdir -p $ROS_ROOT/src

# RUN pip install ament_lint

# ADD *.repos ./
ADD install_deps.sh ./
ADD deepdrive.repos ./
RUN vcs import src < deepdrive.repos
RUN bash install_deps.sh xacro

# TODO: Clean these up. Some may not be needed. Especially for the robot.

RUN bash install_deps.sh \
        foxglove_bridge \
        hls_lfcd_lds_driver \
        joint_state_publisher \
        nav2_bringup \
        navigation2 \
        plotjuggler_ros \
        rmw_cyclonedds_cpp \
        rmw_fastrtps_cpp \
        robot_localization \
        robot_state_publisher \
        ros2bag \
        rosbag2_storage_default_plugins \
        rqt_tf_tree \
        slam_toolbox 

# ADD depthai.repos ./
RUN vcs import src < depthai.repos
RUN bash install_deps.sh depthai

        # depthai
RUN bash install_deps.sh twist_mux usb_cam xacro

RUN bash install_deps.sh depthai_ros_driver

# RUN bash install_deps.sh turtlebot3 
# RUN bash install_deps.sh turtlebot3_gazebo 
# RUN bash install_deps.sh turtlebot3_msgs 
        # depthai_ros_driver \
        # gazebo_plugins \
        # gazebo_ros \
        # gazebo_ros_pkgs \

#   Could not find a package configuration file provided by "absl" with any of
#   the following names:

#     abslConfig.cmake
#     absl-config.cmake

#   Add the installation prefix of "absl" to CMAKE_PREFIX_PATH or set
#   "absl_DIR" to a directory containing one of the above files.  If "absl"
#   provides a separate development package or SDK, be sure it has been
#   installed.
# ENV CMAKE_PREFIX_PATH


WORKDIR $ROS2_WS
COPY src $ROS2_WS/src/deepdrive
RUN bash -c "source install/setup.bash && colcon build --symlink-install"

# RUN bash /opt/ros/humble/install_deps.sh libgazebo_ros_diff_drive
# libgazebo_ros_imu_sensor.so
# libgazebo_ros_ray_sensor.so
# libgazebo_ros_diff_drive.so
# libgazebo_ros_camera.so

# gazebo_plugins
