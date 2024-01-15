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
		-v /etc/timezone:/etc/timezone:ro \
		-v /etc/localtime:/etc/localtime:ro \
		-v ${PWD}/src:${ROS_ROOT}/src/${ROS_PACKAGE} \
		${ROS_PACKAGE} bash

		# TODO: See if we need these
		# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		# -v ${HOME}/.Xauthority:/root/.Xauthority:ro \
		# -v ${PWD}/.session.yml:/root/.session.yml \
		# -v ${PWD}/.tmux.conf:/root/.tmux.conf \

.PHONY: dockersim
dockersim: 
	docker build -t ${ROS_PACKAGE}-sim -f Dockerfile.simulator .

.PHONY: dockersimshell
dockersimshell: dockersim
	xhost +local:docker
	docker run \
		--network=host \
		--runtime nvidia \
		-it \
		--name deepdrive-sim \
		--rm \
		-v ${PWD}/src:${ROS_ROOT}/src/${ROS_PACKAGE} \
		--privileged \
		--gpus=all \
		-e DISPLAY=${DISPLAY} \
		-e PYTHONBUFFERED=1 \
		-v /etc/timezone:/etc/timezone:ro \
		-v /etc/localtime:/etc/localtime:ro \
		--device=/dev/bus/usb:/dev/bus/usb \
		${ROS_PACKAGE}-sim tmux

		# TODO: See if we need these
		# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		# -v ${HOME}/.Xauthority:/root/.Xauthority:ro \
		# -v ${PWD}/.session.yml:/root/.session.yml \
		# -v ${PWD}/.tmux.conf:/root/.tmux.conf \


# Host networking not currently working for x11 (novnc)
# --network=host 