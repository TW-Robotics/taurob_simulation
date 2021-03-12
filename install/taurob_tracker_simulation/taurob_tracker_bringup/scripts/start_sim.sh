#!/bin/bash

declare -A worlds
worlds["1"]="small"
worlds["2"]="medium"
worlds["3"]="large"

OUTDOOR="false"
CARTOGRAPHER="false"
SIZE="1"
OBSTACLES="false"
GUI="false"
GPU="false"
POSE=[""]


usage(){
    echo -e "Usage: $0  [-OPTIONS]\n    OPTIONS:
          [-s <int> (1-3) default: 1 ... set size of generated map, 1=small 3=large]
          [-o ... start outdoor environments]
          [-c ... start cartographer docker image]
          [-b ... simulation with obstacles]
          [-g ... start with gazebo GUI]
          [-G ... start with GPU support]"
    exit 255
}

cleanup(){
    # docker stop $(docker ps -a -q)  # stop cartographer container
    killall -9 gzserver > /dev/null 2>&1
    killall -9 gzclient > /dev/null 2>&1
    echo -e "Finished cleanup"
}

main(){
    path="$(rospack find taurob_tracker_gazebo)/worlds/"
    if [ "$OUTDOOR" == "true" ]; then
        path="$path""Outdoor_Worlds/""${worlds[$SIZE]}"
    else
        path="$path""Indoor_Worlds/"
        if [ $SIZE -eq 1 ]; then
            path="$path""fhtw"
        else
            path="$path""akw"
            POSE="x_pos:=-2.384 y_pos:=-17.147 yaw:=3.1415"
            IFS=" " read -r -a POSE <<< "$POSE"
        fi
    fi
    if [ "$OBSTACLES" == "true" ]; then 
        path="$path""_with_objects"
    fi
    path="$path".world
    echo "path= $path"

    if [ "$CARTOGRAPHER" == "true" ]; then
        echo -e "\e[32mCurrently not implemented\e[0m"
        # echo -e "Using Cartographer docker image as ROS_MASTER"
        # gnome-terminal --tab -e 'docker run -it --rm --name fhtw_cartographer fhtw/cartographer:latest roslaunch cartographer_ros 2d_slamming.launch'  > /dev/null 2>&1
        # sleep 2
        # setupDockerROS
        # echo -e "\e[32mroslaunch taurob_tracker_bringup bringup.launch path_to_world:="$path" gui:="$GUI" outdoor:="$OUTDOOR"\e[0m"
        # roslaunch taurob_tracker_bringup bringup.launch path_to_world:="$path" gui:="$GUI" outdoor:="$OUTDOOR" --wait
        cleanup
        exit 0
    else
        echo -e "Using localhost as ROS_MASTER"
        # shellcheck disable=SC2145   # I know how fixed_"$@" behaves and it's correct!
        echo -e "\e[32mroslaunch taurob_tracker_bringup bringup.launch path_to_world:=$path gui:=$GUI outdoor:=$OUTDOOR gpu:=$GPU ${POSE[@]} \e[0m"
        sleep 5
        roslaunch taurob_tracker_bringup bringup.launch path_to_world:="$path" gui:="$GUI" outdoor:="$OUTDOOR" gpu:="$GPU" "${POSE[@]}"
        cleanup
        exit 0
    fi

}

while getopts "ocbgGs:" arg
do
    case "$arg" in
        o)  OUTDOOR="true";;
        b)  OBSTACLES="true";;
        c)  CARTOGRAPHER="true";;
        g)  GUI="true";;
        G)  GPU="true";;
        s)  SIZE="$OPTARG"
            ;;
        *)  echo -e "\e[31mInvalid option\e[0m"
            usage ;;
    esac
done


main 
