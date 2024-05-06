#!/usr/bin/env bash

SCRIPT_PATH=$(dirname "$0")
DOCKER_USER="ros"
docker run -it \
	--user $DOCKER_USER --network=host --ipc=host -v $SCRIPT_PATH/../..:/home/$DOCKER_USER/ros2_ws \
	-v $HOME/.gitconfig:/home/$DOCKER_USER/.gitconfig \
	-e DISPLAY=$DISPLAY \
	forg_bot:dev
