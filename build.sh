#!/bin/bash

usage() 
{
    printf "Usage: \n\t%s [-v <cpu | gpu (default)> ] [-h print this message]\n" "$0" 1>&2
    exit 1
}

ROS_DISTRO="melodic" # supported versions: ["melodic"]
VERSION="-gpu-ml"       # supported versions: ["", "-gpu-ml"]
opts="v:h"

while getopts ${opts} arg; do
    case ${arg} in
    v)
        if [ "${OPTARG}" == "gpu" ]; then
            VERSION="-gpu-ml"
            CUDA="on"
        elif [ "${OPTARG}" == "cpu" ]; then
            VERSION=""
            CUDA="off"
        else
            echo "Not support -v argument"
            usage
        fi
        echo "Version = >$VERSION<"
        ;;
    h)
        usage
        ;;
    ?)
        echo "Not support argument"
        usage
        ;;
    esac
done




build() 
{
    if [ "$CUDA" != "on" ]; then
        echo "Building with CUDA support"
    else
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
    if [ $0 -eq 0 ]; then
    # Changes in files depending on selected distro and version #
    sed -i "s/$ROS_DISTRO/ROS_DISTRO/" ./ros_entrypoint.sh || true
        if [ "$CUDA" == "on" ]; then
            sed -i "s#--privileged#--privileged\n\t--gpus all #" ./run_docker.sh || true
            sed -i "s#--privileged#--privileged \\\#" ./run_docker.sh || true
            sed -i "s/"$ROS_DISTRO"_taurob_simulation/"$ROS_DISTRO""$VERSION"_taurob_simulation/" ./run_docker.sh || true
            exit 0
        else
            
        fi
    else 
        echo -e "\e[31mSomething went wrong during build\e[0m" 
        exit 1
    fi
}

main() 
{
    build
}

main