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
RUN sudo apt update && sudo apt install -y git xterm curl gosu tmux vim less\
    bash-completion libboost-all-dev clang-format bc\
    ros-$ROS_DISTRO-dwa-local-planner ros-$ROS_DISTRO-costmap-2d ros-$ROS_DISTRO-hector-gazebo* ros-$ROS_DISTRO-global-planner\
    ros-$ROS_DISTRO-turtlebot3* ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-pid ros-$ROS_DISTRO-rosdoc-lite ros-$ROS_DISTRO-gmapping\
    ros-$ROS_DISTRO-rqt* ros-melodic-robot-localization ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-hector-xacro-tools ros-$ROS_DISTRO-hector-gazebo-plugins ros-$ROS_DISTRO-gps-common \
    ros-$ROS_DISTRO-robot-pose-ekf ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-position-controllers ros-$ROS_DISTRO-velodyne-gazebo-plugins ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-octomap* ros-$ROS_DISTRO-hector-sensors-description ros-$ROS_DISTRO-teleop-twist-keyboard --no-install-recommends

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
#----------#

# Set up Simulation Environment #
RUN mkdir -p /home/$USERNAME/git/gazebo_sim && cd /home/$USERNAME/git/gazebo_sim \
	&& git clone https://github.com/tu-darmstadt-ros-pkg/gazebo_ros_control_select_joints.git \
	&& git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
RUN cd /home/$USERNAME/catkin_ws/src && ln -s /home/$USERNAME/git/gazebo_sim/* . 


# Set up taurob simulation 
RUN cd /home/$USERNAME/catkin_ws/src && /ros_entrypoint.sh catkin_init_workspace 
COPY ./install/taurob_tracker_simulation /home/$USERNAME/catkin_ws/src/taurob_tracker_simulation
COPY ./install/mapping /home/$USERNAME/catkin_ws/src/fhtw_pkgs/mapping
COPY ./install/navigation /home/$USERNAME/catkin_ws/src/fhtw_pkgs/navigation
COPY ./install/perception /home/$USERNAME/catkin_ws/src/fhtw_pkgs/perception
RUN if [ ${CUDA} == "on" ]; then sed -i 's#arg name="gpu"       default="false"#arg name="gpu"       default="true"#g' /home/$USERNAME/catkin_ws/src/taurob_tracker_simulation/taurob_tracker_bringup/launch/bringup.launch; fi
RUN if [ ${CUDA} == "on" ]; then sed -i 's#GPU="false"#GPU="true"#g' /home/$USERNAME/catkin_ws/src/taurob_tracker_simulation/taurob_tracker_bringup/scripts/start_sim.sh; fi
RUN cd /home/$USERNAME/ && sudo chown -R fhtw_user:fhtw_user catkin_ws
RUN cd /home/ && sudo chown -R fhtw_user:fhtw_user $USERNAME
USER fhtw_user
RUN cd /home/$USERNAME/catkin_ws/ && rosdep install --from-paths src --ignore-src -r -y
USER root

# Install dependencies #
# Detection
RUN sudo apt update && sudo apt install -y tesseract-ocr tesseract-ocr-deu libleptonica-dev libtesseract-dev python-catkin-tools
# Openpose
RUN if [ ${CUDA} == "on" ]; then sudo apt install -y libgoogle-glog-dev cuda-cublas* libatlas-base-dev libatlas3-base liblapacke-dev  build-essential graphviz libboost-filesystem-dev libboost-python-dev libboost-system-dev libboost-thread-dev libgflags-dev libgoogle-glog-dev \
    libhdf5-serial-dev libopenblas-dev python-virtualenv wget  libncurses5-dev libncursesw5-dev\
    apt-transport-https ca-certificates gnupg software-properties-common wget && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null\ 
    && sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' && sudo apt update && sudo apt install -y cmake; fi
USER fhtw_user
RUN cd /home/$USERNAME/git && if [ ${CUDA} == "on" ]; then git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose; fi \
    && git clone --recursive https://github.com/leggedrobotics/darknet_ros \
    && if [ ${CUDA} == "on" ]; then cd openpose && git checkout tags/v1.7.0 && git submodule update --init --recursive --remote && mkdir build; fi
# Manually set CUDA and NVCC Version to build openpose and 3d party libs
RUN if [ ${CUDA} == "on" ]; then sed -i 's/-DCUDA_ARCH_NAME=${CUDA_ARCH}/-DCUDA_ARCH_NAME=Turing/g' /home/$USERNAME/git/openpose/CMakeLists.txt ; fi
RUN if [ ${CUDA} == "on" ]; then sed -i "/ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)/a   set(CUDA_gpu_detect_output 7.5)" /home/$USERNAME/git/openpose/cmake/Cuda.cmake; fi
RUN if [ ${CUDA} == "on" ]; then cd /home/$USERNAME/git/openpose && cd build && cmake .. && make -j && sudo make install; fi
RUN git clone https://github.com/ravijo/ros_openpose /home/$USERNAME/catkin_ws/src/ros_openpose && sed -i "/find_package(OpenMP)/a   find_package(Threads REQUIRED)" /home/$USERNAME/catkin_ws/src/ros_openpose/CMakeLists.txt
RUN cd /home/$USERNAME/catkin_ws/src/ && ln -s  /home/$USERNAME/git/darknet_ros /home/$USERNAME/catkin_ws/src/ && cd /home/$USERNAME/catkin_ws/ && rm -rf build devel \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build -c -DCMAKE_BUILD_TYPE=Release"; exit 0

# Cartographer
RUN sudo apt-get install -y python-wstool python-rosdep ninja-build stow
RUN mkdir -p /home/$USERNAME/cartographer_ws/ && cd /home/$USERNAME/cartographer_ws && wstool init src && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
   &&  wstool update -t src && rosdep install --from-paths src --ignore-src --rosdistro=melodic -y && bash src/cartographer/scripts/install_abseil.sh \
   && git clone --branch v3.4.1 https://github.com/google/protobuf.git \
   && cd protobuf && mkdir build && cd build \ 
   && cmake -G Ninja \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -Dprotobuf_BUILD_TESTS=OFF \
    -DCMAKE_INSTALL_PREFIX=../install \
    ../cmake && ninja && ninja install
RUN cd /home/$USERNAME/cartographer_ws/ && source /home/fhtw_user/catkin_ws/devel/setup.bash && source /opt/ros/melodic/setup.bash &&  catkin_make_isolated --install --use-ninja \
  -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH};${PWD}/install_isolated;${PWD}/protobuf/install"
RUN source /home/fhtw_user/cartographer_ws/install_isolated/setup.bash && rm -r /home/fhtw_user/catkin_ws/devel && cd /home/fhtw_user/catkin_ws && catkin build -c -DCMAKE_BUILD_TYPE=Release && source devel/setup.bash; exit 0
USER root
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]