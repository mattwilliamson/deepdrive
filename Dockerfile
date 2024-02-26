# FROM dustynv/ros:humble-pytorch-l4t-r35.4.1
FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

# NOVNC remote desktop Nabbed from https://github.com/theasp/docker-novnc/blob/master/Dockerfile

# # Install git, supervisor, VNC, & X11 packages
# RUN apt-get update && \
#     apt-get install -y \
#       bash \
#       fluxbox \
#       git \
#       net-tools \
#       novnc \
#       supervisor \
#       x11vnc \
#       xterm \
#       xvfb && \
#     rm -rf /var/lib/apt/lists/

# # Setup demo environment variables
# ENV HOME=/root \
#     DEBIAN_FRONTEND=noninteractive \
#     LANG=en_US.UTF-8 \
#     LANGUAGE=en_US.UTF-8 \
#     LC_ALL=C.UTF-8 \
#     DISPLAY=:0.0 \
#     DISPLAY_WIDTH=1024 \
#     DISPLAY_HEIGHT=768 \
#     RUN_XTERM=yes \
#     RUN_FLUXBOX=yes
    
# COPY novnc /novnc
# # CMD ["/novnc/entrypoint.sh"]
# EXPOSE 8080


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
    DISPLAY_HEIGHT=900 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR $ROS_ROOT

RUN echo echo SOURCING ROS $ROS2_WS/install/setup.bash >> ~/.bashrc && \
    echo source $ROS2_WS/install/setup.bash >> ~/.bashrc && \
    mkdir -p $ROS2_WS/src \
    mkdir -p $ROS_ROOT/src

RUN cd /tmp && \
    git clone --recursive https://github.com/luxonis/depthai-core.git --branch main && \
    cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build depthai-core/build --target install && \
    rm -rf /tmp/depthai-core

# ADD *.repos ./
ADD install_deps.sh ./
ADD deepdrive.repos ./
RUN vcs import src < deepdrive.repos

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
        slam_toolbox \
        twist_mux \
        usb_cam \
        xacro \
        gazebo_plugins \
        gazebo_ros_pkgs \
        rtabmap_ros \
        foxglove_msgs \
        ros2_control \
        ros2_controllers \
        control_toolbox \
        realtime_tools \
        control_msgs \
        ros2_control_demos

# Install nav2
# RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#         && git clone https://github.com/ros-planning/navigation2.git && cd navigation2 && git checkout ec49c2772a0926c86ca83a4933c664744712e2e9 && cd .. \
#         && git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && cd BehaviorTree.CPP && git checkout a363bdcae88350bc748598a7d2950e300859469c && cd .. \
#         && source ${ROS_ROOT}/setup.bash && cd ${ROS_ROOT} \
#         && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} \
#         && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to-regex nav2* --packages-ignore nav2_system_tests \
#         && rm -Rf src build log \
#     && rm -rf /var/lib/apt/lists/* \
#     && apt-get clean

# depthai_examples: No definition of [foxglove_msgs] for OS version [focal]
# nav2_mppi_controller: [xsimd] defined as "not available" for OS version [focal]
# depthai_ros_driver: No definition of [image_transport_plugins] for OS version [focal]


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
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        zsh && \
    rm -rf /var/lib/apt/lists/
RUN pip3 install setuptools==58.2.0 && \
    pip3 install --upgrade pip

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


# micro ros
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git /usr/src/pico-sdk
ENV PICO_SDK_PATH=/usr/src/pico-sdk

ENV UROS_WS=/uros_ws
WORKDIR $UROS_WS
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN source /opt/ros/${ROS_DISTRO}/install/setup.sh && \
    apt-get update && \
    rosdep update && \
    rm -rf /opt/ros/humble/build/pluginlib/pluginlib_enable_plugin_testing && \
    rosdep install --from-paths src --ignore-src -y 
RUN source /opt/ros/${ROS_DISTRO}/install/setup.sh && \
    colcon build && \
    source $UROS_WS/install/setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200

# Compile example:
# git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git -b $ROS_DISTRO src/micro_ros_raspberrypi_pico_sdk
# cd micro_ros_raspberrypi_pico_sdk
# mkdir build
# cd build
# cmake ..
# make

# RUN git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble src/micro-ROS-Agent



# Install JetsonGPIO
# TODO: deprecate in favor of microcontroller
WORKDIR /usr/src
RUN git clone https://github.com/pjueon/JetsonGPIO.git && \
    cd JetsonGPIO && \
    mkdir build && \
    cd build && \
    cmake .. -DJETSON_GPIO_POST_INSTALL=OFF && \
    cmake --build . --target install

RUN echo "source /root/.ros2" >> /root/.bashrc
RUN echo "source /root/.ros2" >> /root/.zshrc
RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.ros2
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.ros2
RUN echo "source $ROS_ROOT/install/setup.sh" >> /root/.ros2
RUN echo "source $UROS_WS/install/setup.sh" >> /root/.ros2
RUN echo "source $ROS2_WS/install/setup.sh" >> /root/.ros2
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.ros2

RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.ros2
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.ros2
RUN echo 'alias ssetup="source $ROS2_WS/install/setup.sh"' >> /root/.ros2
RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> /root/.ros2
RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> /root/.ros2
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.ros2

# RUN echo "autoload -U bashcompinit" >> /root/.ros2
# RUN echo "bashcompinit" >> /root/.ros2
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.ros2
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.ros2

CMD [ "tmuxinator", "start", "-p", "/root/.session.yml" ]
# CMD [ "bash" ]

ENV DEEPDRIVE_MODEL=deepdrive

COPY .session.yml /root/.session.yml
COPY .tmux.conf /root/.tmux.conf



# ----------------------------------------------------------------------------


# Add ROS2 source code
WORKDIR $ROS2_WS

# COPY src/deepdrive_hardware/requirements.txt src/deepdrive_hardware/requirements.txt
# RUN pip3 install -r src/deepdrive_hardware/requirements.txt

# COPY src/deepdrive_vision/requirements.txt src/deepdrive_vision/requirements.txt
# RUN pip3 install -r src/deepdrive_vision/requirements.txt

COPY src/imu_bno08x/requirements.txt src/imu_bno08x/requirements.txt
RUN pip3 install -r src/imu_bno08x/requirements.txt

COPY src/differential_drive/requirements.txt src/differential_drive/requirements.txt
RUN pip3 install -r src/differential_drive/requirements.txt

COPY src ./src/


VOLUME /tmp


RUN bash -c "source $ROS_ROOT/install/setup.bash && colcon build --symlink-install"
