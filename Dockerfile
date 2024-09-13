FROM dustynv/ros:humble-llm-r36.3.0

VOLUME /tmp

ENV DEBIAN_FRONTEND=noninteractive

# Common dependencies & tools
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        vim \
        git \
        portaudio19-dev \
        python3-pip \
        python3-pydantic \
        wget \
        curl \
        xorg-dev \
        xtl-dev \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        zsh \
        build-essential \
        cmake \
        libeigen3-dev \
        libtbb-dev \
        pybind11-dev \
        net-tools \
        ninja-build && \
    rm -rf /var/lib/apt/lists/

#  TODO: Needed this specific version for some reason?
# RUN pip3 install setuptools==58.2.0 && \
    # pip3 install --upgrade pip

RUN pip3 install --upgrade pip setuptools

# RUN ROS_WORKSPACE=/ros2_workspace /ros2_install.sh https://github.com/dusty-nv/ros_deep_learning

# Main prerequisites
WORKDIR /opt/ros/${ROS_DISTRO}

# git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs.git
# colcon build --packages-select-regex '.*ffmpeg.*'
# colcon build --packages-select-regex '.*depthai.*'

# cd /opt/ros/${ROS_DISTRO}/src/
# git clone https://github.com/ros-perception/imu_pipeline.git
# source /opt/ros/${ROS_DISTRO}/install/setup.sh
# colcon build --packages-select-regex '.*imu.*'

ADD *.repos ./
ADD install_deps.sh ./
RUN chmod +x install_deps.sh
ADD deepdrive.repos ./
RUN mkdir src && \
    vcs import src < deepdrive.repos
    
RUN ./install_deps.sh \
        foxglove_bridge foxglove_msgs \
        joint_state_publisher \
        nav2_bringup \
        navigation2 \
        rmw_cyclonedds_cpp \
        rmw_fastrtps_cpp \
        robot_localization \
        robot_state_publisher \
        ros2bag \
        rosbag2_storage_default_plugins \
        slam_toolbox \
        twist_mux \
        xacro \
        ros2_control \
        ros2_controllers \
        control_toolbox \
        realtime_tools \
        control_msgs
        

    # rtabmap_ros \
    # gscam \
    # ros2_control_demos


# DepthAI (OAK-D LITE stereo camera)
# ENV DEPTHAI_SRC=/opt/depthai-core
WORKDIR /opt/
RUN apt-get update && \
    apt install libpcl-dev -y && \
    rm -rf /var/lib/apt/lists/
RUN git clone --recursive https://github.com/luxonis/depthai-core.git --branch main && \
    cd depthai-core && \
    git submodule update --init --recursive && \
    cmake -S. -Bbuild -D'BUILD_SHARED_LIBS=ON' -D'CMAKE_INSTALL_PREFIX=/usr/local' && \
    cmake --build build --target install


# MicroROS
ENV UROS_WS=/uros_ws
WORKDIR $UROS_WS
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN source /opt/ros/${ROS_DISTRO}/install/setup.sh && \
    apt-get update && \
    rosdep update && \
    rm -rf /opt/ros/$ROS_DISTRO/build/pluginlib/pluginlib_enable_plugin_testing && \
    rosdep install --from-paths src --ignore-src -y && \
    source /opt/ros/${ROS_DISTRO}/install/setup.sh && \
    colcon build && \
    source $UROS_WS/install/setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh


# Main Code

ENV ROS2_WS=/ros_ws \
    ROS_PACKAGE=deepdrive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR $ROS2_WS

RUN echo source $UROS_WS/install/setup.sh >> ~/.bashrc && \
    echo echo SOURCING ROS $ROS2_WS/install/setup.bash >> ~/.bashrc && \
    echo source $ROS2_WS/install/setup.bash >> ~/.bashrc && \
    mkdir -p $ROS2_WS/src

RUN cat <<EOF >> ~/.bash_history
colcon build --symlink-install --packages-skip-regex '.*gazebo.*'
source $ROS2_WS/install/setup.bash
ros2 launch deepdrive_bringup robot.launch.py
ros2 run deepdrive_teleop teleop_keyboard
ros2 launch deepdrive_nav2_bringup navigation_launch.py use_sim_time:=False params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml
ros2 launch deepdrive_nav2_bringup slam_launch.py use_sim_time:=False params_file:=src/deepdrive_nav2_bringup/params/nav2_params.yaml
EOF

