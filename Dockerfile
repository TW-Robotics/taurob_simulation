ARG ROS_DISTRO
ARG VERSION
FROM georgno/fhtw-ros:${ROS_DISTRO}${VERSION}
ARG CUDA
ENV USERNAME fhtw_user
RUN echo "CUDA:   ${CUDA}" && sleep 5

SHELL ["/bin/bash", "-o", "pipefail", "-c"]



#
# Install tools and libraries required
#
USER root
RUN sudo apt update && sudo apt install -y git xterm curl gosu tmux vim xterm less\
    bash-completion libboost-all-dev clang-format bc\
    ros-$ROS_DISTRO-rosdoc-lite ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-hector-xacro-tools ros-$ROS_DISTRO-hector-gazebo-plugins ros-$ROS_DISTRO-gps-common \
    ros-$ROS_DISTRO-robot-pose-ekf ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-position-controllers ros-$ROS_DISTRO-velodyne-gazebo-plugins ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-hector-sensors-description ros-$ROS_DISTRO-teleop-twist-keyboard --no-install-recommends

#-------------#
# USER fhtw_user
#--GAZEBO--#
# Download models
RUN mkdir -p /home/$USERNAME/.gazebo/models && git clone https://github.com/osrf/gazebo_models /home/$USERNAME/.gazebo/models/


# # Upgrade GAZEBO to support RAY-GPU
RUN if [ ${CUDA} == "on" ]; then sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'; fi
RUN if [ ${CUDA} == "on" ]; then sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743; fi
RUN if [ ${CUDA} == "on" ]; then sudo apt update ; fi
RUN if [ ${CUDA} == "on" ]; then sudo apt-get --only-upgrade install -y gazebo9 gazebo9-common libgazebo9 libgazebo9-dev libignition-math2*; fi
RUN if [ ${CUDA} == "on" ]; then sudo apt upgrade -y libignition-math2; fi


USER fhtw_user
RUN rosdep update

USER root
# #----------#
# # Set up Simulation Environment #
RUN mkdir -p /home/$USERNAME/git/gazebo_sim && cd /home/$USERNAME/git/gazebo_sim \
	&& git clone https://github.com/tu-darmstadt-ros-pkg/gazebo_ros_control_select_joints.git \
	&& git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
RUN cd /home/$USERNAME/catkin_ws/src && ln -s /home/$USERNAME/git/gazebo_sim/* . 

RUN echo "1"
# # Set up taurob simulation 
RUN cd /home/$USERNAME/catkin_ws/src && /ros_entrypoint.sh catkin_init_workspace 
COPY ./install/taurob_tracker_simulation /home/$USERNAME/catkin_ws/src/taurob_tracker_simulation
RUN if [ ${CUDA} == "on" ]; then sed -i 'g#arg name="gpu"       default="false"#arg name="gpu"       default="true"#g' /home/$USERNAME/catkin_ws/src/taurob_tracker_simulation/taurob_tracker_bringup/launch/*.launch; fi
RUN cd /home/$USERNAME/ && sudo chown -R fhtw_user:fhtw_user catkin_ws
RUN cd /home/ && sudo chown -R fhtw_user:fhtw_user $USERNAME


ENTRYPOINT ["/ros_entrypoint.sh"]