#!/usr/bin/env bash

SCRIPT_PATH=$(dirname "$0")
DOCKER_USER="ros"

CONTAINER_NAME="forg_bot"

temp=$(mktemp)
echo "name=\"$CONTAINER_NAME\"" >>"$temp"

docker run -it \
	--user $DOCKER_USER --network=host --ipc=host -v $SCRIPT_PATH/../..:/home/$DOCKER_USER/ros2_ws \
	-v $HOME/.gitconfig:/home/$DOCKER_USER/.gitconfig \
	-e DISPLAY=$DISPLAY \
	--name jumpy_frog \
	--rm \
	-v $temp:/run/.containerenv \
	forg_bot:dev

rm $temp
