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
# TODO: Move this above all setup_ws.sh and consolidate those
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

ENV ROS2_WS=/ros_ws \
    ROS_PACKAGE=deepdrive \
    DISPLAY_WIDTH=1600 \
    DISPLAY_HEIGHT=900

WORKDIR $ROS_ROOT
# ADD *.repos ./
ADD setup_ws.sh ./
RUN bash setup_ws.sh \
        xacro \
	    gazebo_ros \
	    robot_localization \
        gazebo_plugins

        


# WORKDIR $ROS2_WS
# COPY src $ROS2_WS/src/deepdrive
# RUN bash -c "source install/setup.bash && colcon build --symlink-install"

# RUN bash /opt/ros/humble/setup_ws.sh libgazebo_ros_diff_drive
# libgazebo_ros_imu_sensor.so
# libgazebo_ros_ray_sensor.so
# libgazebo_ros_diff_drive.so
# libgazebo_ros_camera.so

# gazebo_plugins