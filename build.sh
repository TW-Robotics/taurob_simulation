#!/bin/bash

usage()
{
    printf "Usage: \n\t%s [-v <cpu | gpu (default)> ] [-h print this message]\n" "$0" 1>&2
    exit 1
}

ROS_DISTRO="melodic" # supported versions: ["melodic"]
VERSION="-gpu-ml"       # supported versions: ["", "gpu", "-gpu-ml"]
LABEL="gpu"
REPO="georgno/fhtw_taut"
opts="v:h"

while getopts ${opts} arg; do
    case ${arg} in
    v)
        if [ "${OPTARG}" == "gpu" ]; then
            VERSION="-gpu-ml"
            CUDA="on"
            LABEL="${OPTARG}"
        elif [ "${OPTARG}" == "cpu" ]; then
            VERSION=""
            CUDA="off"
            LABEL="${OPTARG}"
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
    if [ "$CUDA" == "on" ]; then
        echo "Building with CUDA support"
    else
        echo "Building without CUDA support"
    fi
    sleep 5
    sed -i "s/ROS_DISTRO/$ROS_DISTRO/" ./ros_entrypoint.sh

    docker pull georgno/fhtw-ros:"$ROS_DISTRO""$VERSION"
    docker build \
        --rm \
        --tag $REPO:$LABEL \
        --build-arg ROS_DISTRO="$ROS_DISTRO" \
        --build-arg VERSION="$VERSION" \
        --build-arg CUDA="$CUDA" \
        --file Dockerfile .
    DOCKER_BUILD_FLAG=$?
    # Changes in files depending on selected distro and version #
    sed -i "s/$ROS_DISTRO/ROS_DISTRO/" ./ros_entrypoint.sh || true
    if [ "$CUDA" == "on" ]; then
        if ! grep -rq "gpus" run_docker.sh; then
            sed -i "s#--privileged#--privileged\n\t--gpus all #" ./run_docker.sh || true
            sed -i "s#--privileged#--privileged \\\#" ./run_docker.sh || true
        fi
        echo -e "\e[32mBuild with CUDA support\e[0m"
    else
        echo -e "\e[32mBuild without CUDA support\e[0m"
        sed  -i "/--gpus all  \\\/d" ./run_docker.sh || true
    fi
    sed -i "s/georgno\/fhtw_taut:LABEL/georgno\/fhtw_taut:$LABEL/" ./run_docker.sh || true

    # Change Docker hook variables#
    if [ $DOCKER_BUILD_FLAG -eq 0 ]; then
        sed -i "s/LABEL=\"LABEL\"/LABEL=\"$LABEL\"/" ./hooks/build
        sed -i "s/VERSION=\"VERSION\"/VERSION=\"$VERSION\"/" ./hooks/build
        sed -i "s/CUDA=\"CUDA\"/CUDA=\"$CUDA\"/" ./hooks/build
    fi

}

main()
{
    build
}

main