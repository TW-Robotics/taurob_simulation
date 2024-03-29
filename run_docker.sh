#!/bin/sh
USER_ID="$(id -u)"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
SHARED_DOCKER_DIR=/home/fhtw_user/catkin_ws/src/fhtw
SHARED_HOST_DIR=$(pwd)/src
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw"

if [ -z $1  ]; then
    LABEL=cpu
else
    LABEL=gpu
fi
mkdir -p "$SHARED_HOST_DIR"


echo -e "\e[32mMounting fodler:
    $SHARED_HOST_DIR    to
    $SHARED_DOCKER_DIR\e[0m"


docker run \
    -it --rm \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    --name="taurob_sim" \
    georgno/fhtw-tracker-sim:$LABEL \
    tmux

