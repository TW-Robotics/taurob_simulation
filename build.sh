#!/bin/bash
ROS_DISTRO="melodic"    # supported versions: ["melodic"]
VERSION="-gpu"   # supported versions: ["", "-gpu", "-gpu-ml"]
if [ "$VERSION" != "" ]; then
    CUDA="on"
    echo "Building with CUDA support"
else
    CUDA="off"
    echo "Building without CUDA support"
fi

sed -i "s/ROS_DISTRO/$ROS_DISTRO/" ./ros_entrypoint.sh

docker pull georgno/fhtw-ros:"$ROS_DISTRO""$VERSION"
docker build \
    --rm \
    --tag "$ROS_DISTRO""$VERSION"_taurob_simulation \
    --build-arg ROS_DISTRO="$ROS_DISTRO" \
    --build-arg VERSION="$VERSION" \
    --build-arg CUDA="$CUDA" \
    --file Dockerfile .

# Changes in files depending on selected distro and version #
sed -i "s/$ROS_DISTRO/ROS_DISTRO/" ./ros_entrypoint.sh || true
if [ "$CUDA" == "on" ]; then
    sed -i "s#--privileged#--privileged\n\t--gpus all #" ./run_docker.sh || true
    sed -i "s#--privileged#--privileged \\\#" ./run_docker.sh || true
    sed -i "s/"$ROS_DISTRO"_taurob_simulation/"$ROS_DISTRO""$VERSION"_taurob_simulation/" ./run_docker.sh || true
fi
