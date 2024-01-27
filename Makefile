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
	docker run \
		--network=host \
		--runtime nvidia \
		-it \
		--name deepdrive \
		--rm \
		--privileged \
		--gpus=all \
		-e DISPLAY=${DISPLAY} \
		-e PYTHONBUFFERED=1 \
		-v /dev/:/dev/ \
		-v /tmp:/tmp \
		-v /proc/device-tree/compatible:/proc/device-tree/compatible \
		-v /proc/device-tree/chosen:/proc/device-tree/chosen \
		--device /dev/gpiochip0 \
		-v /etc/timezone:/etc/timezone:ro \
		-v /etc/localtime:/etc/localtime:ro \
		-v ${PWD}/src:${ROS_ROOT}/src/ \
		${ROS_PACKAGE} bash

		# TODO: See if we need these
		# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		# -v ${HOME}/.Xauthority:/root/.Xauthority:ro \
		# -v ${PWD}/.session.yml:/root/.session.yml \
		# -v ${PWD}/.tmux.conf:/root/.tmux.conf \
		# -e JETSON_MODEL_NAME="JETSON_ORIN_NANO" \

.PHONY: dockersim
dockersim: 
	docker build -t ${ROS_PACKAGE}-sim -f Dockerfile.simulator .

.PHONY: dockersimshell
dockersimshell: dockersim
	# xhost +local:docker
	docker run \
		--network=host \
		--runtime nvidia \
		-it \
		--name deepdrive-sim \
		--rm \
		-v ${PWD}/src:${ROS_ROOT}/src/ \
		--privileged \
		--gpus=all \
		-e DISPLAY=${DISPLAY} \
		-e PYTHONBUFFERED=1 \
		-v /etc/timezone:/etc/timezone:ro \
		-v /etc/localtime:/etc/localtime:ro \
		-v /tmp/deepdrive:/tmp \
		-v "/tmp/.gazebo/:/root/.gazebo/" \
		--device=/dev/bus/usb:/dev/bus/usb \
		${ROS_PACKAGE}-sim

		# TODO: See if we need these
		# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		# -v ${HOME}/.Xauthority:/root/.Xauthority:ro \
		# -v ${PWD}/.session.yml:/root/.session.yml \
		# -v ${PWD}/.tmux.conf:/root/.tmux.conf \


# Host networking not currently working for x11 (novnc)
# --network=host 