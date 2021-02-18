#!/bin/bash

ROS_DISTRO="melodic"    # supported versions: ["melodic"]
VERSION=""   # supported versions: ["", "-gpu", "-gpu-ml"]
CUDA="off"

sed -i "s/ROS_DISTRO/$ROS_DISTRO/" ./ros_entrypoint.sh

docker pull georgno/fhtw-ros:"$ROS_DISTRO""$VERSION"
docker build \
    --rm \
    --tag "$ROS_DISTRO""$VERSION"_taurob_simulation \
    --build-arg ROS_DISTRO="$ROS_DISTRO" \
    --build-arg VERSION="$VERSION" \
    --build-arg CUDA="$CUDA" \
    --file Dockerfile .

sed -i "s/$ROS_DISTRO/ROS_DISTRO/" ./ros_entrypoint.sh
