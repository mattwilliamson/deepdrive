ROS_DISTRO=humble
ROS_PACKAGE=deepdrive
ROS_ROOT=/ros_ws

rosdeps:
	rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ./src/${ROS_PACKAGE} > ${ROS_PACKAGE}.rosinstall

.PHONY: docker
docker:
	docker build -t ${ROS_PACKAGE} .

.PHONY: dockershell
dockershell: docker
	docker run --runtime nvidia -it -p 8080:8080 --name deepdrive --rm -v ${PWD}/src:${ROS_ROOT}/src/${ROS_PACKAGE} ${ROS_PACKAGE} bash

# Host networking not currently working for x11 (novnc)
# --network=host 