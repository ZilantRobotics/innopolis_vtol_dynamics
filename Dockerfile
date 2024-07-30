ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="UAV simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/uav_hitl_simulator


# 1. Install basic requirements
RUN apt-get update                                                              &&  \
    apt-get upgrade -y                                                          &&  \
    apt-get install -y  git ros-$ROS_DISTRO-catkin python3-pip

# hack because there is an issue on >=71.0.0 https://github.com/pypa/setuptools/issues/4478
RUN pip install setuptools==68.0.0 importlib-metadata==8.2.0
RUN apt-get install -y python3-catkin-tools

RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install requirements
# 2.1. geographiclib_conversions
COPY uav_dynamics/geographiclib_conversions uav_dynamics/geographiclib_conversions/
RUN ./uav_dynamics/geographiclib_conversions/scripts/install.sh
RUN source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    cd ../../                                                                   &&  \
    git config --global http.sslverify false                                    && \
    catkin build

# 2.2. inno-sim-interface
RUN sudo apt-get install -y ros-$ROS_DISTRO-rosauth                             &&  \
    pip install bson pymongo protobuf Pillow twisted

# 2.3. innopolis_vtol_dynamics
COPY uav_dynamics/uav_hitl_dynamics/install_requirements.sh    uav_dynamics/uav_hitl_dynamics/install_requirements.sh
COPY uav_dynamics/uav_hitl_dynamics/requirements.txt           uav_dynamics/uav_hitl_dynamics/requirements.txt
RUN apt-get update && \
    uav_dynamics/uav_hitl_dynamics/install_requirements.sh

# 2.4 tools/can
COPY scripts/tools scripts/tools
RUN ./scripts/tools/can/install.sh --yes

# 2.5. communicators
COPY communicators/mavlink_communicator/                        communicators/mavlink_communicator/
COPY communicators/uavcan_communicator/                         communicators/uavcan_communicator/
COPY communicators/cyphal_communicator/scripts/config.sh        communicators/cyphal_communicator/scripts/config.sh
COPY communicators/cyphal_communicator/install_requirements.sh  communicators/cyphal_communicator/install_requirements.sh
COPY communicators/cyphal_communicator/requirements.txt         communicators/cyphal_communicator/requirements.txt
COPY communicators/cyphal_communicator/compile_dsdl.sh          communicators/cyphal_communicator/compile_dsdl.sh
RUN ./communicators/uavcan_communicator/scripts/install_requirements.sh         &&  \
    ./communicators/uavcan_communicator/scripts/install_libuavcan.sh
RUN ./communicators/cyphal_communicator/install_requirements.sh                 &&  \
    ./communicators/cyphal_communicator/compile_dsdl.sh

# 3. Copy the source files
COPY inno_sim_interface/ inno_sim_interface/
COPY catkin_build.sh catkin_build.sh

COPY uav_dynamics/uav_hitl_dynamics/libs               uav_dynamics/uav_hitl_dynamics/libs
COPY uav_dynamics/uav_hitl_dynamics/meshes             uav_dynamics/uav_hitl_dynamics/meshes
COPY uav_dynamics/uav_hitl_dynamics/src                uav_dynamics/uav_hitl_dynamics/src
COPY uav_dynamics/uav_hitl_dynamics/tests              uav_dynamics/uav_hitl_dynamics/tests
COPY uav_dynamics/uav_hitl_dynamics/urdf               uav_dynamics/uav_hitl_dynamics/urdf
COPY uav_dynamics/uav_hitl_dynamics/CMakeLists.txt     uav_dynamics/uav_hitl_dynamics/CMakeLists.txt
COPY uav_dynamics/uav_hitl_dynamics/package.xml        uav_dynamics/uav_hitl_dynamics/package.xml
COPY communicators/cyphal_communicator/src              communicators/cyphal_communicator/src
COPY communicators/cyphal_communicator/Libs             communicators/cyphal_communicator/Libs
COPY communicators/cyphal_communicator/CMakeLists.txt   communicators/cyphal_communicator/CMakeLists.txt
COPY communicators/cyphal_communicator/package.xml      communicators/cyphal_communicator/package.xml

# 4. Build ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    cd ../../                                                                   &&  \
    git config --global http.sslverify false                                    && \
    catkin build

# 5. Copy configs, scripts, etc
COPY uav_dynamics/uav_hitl_dynamics/scripts/           uav_dynamics/uav_hitl_dynamics/scripts
COPY uav_dynamics/uav_hitl_dynamics/launch/            uav_dynamics/uav_hitl_dynamics/launch
COPY uav_dynamics/uav_hitl_dynamics/config/            uav_dynamics/uav_hitl_dynamics/config
COPY uav_dynamics/uav_hitl_dynamics/catkin_test.sh     uav_dynamics/uav_hitl_dynamics/catkin_test.sh
COPY scripts/ scripts/
COPY communicators/cyphal_communicator/                 communicators/cyphal_communicator/

RUN echo source scripts/run_sim.sh ros >> ~/.bashrc

CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash &&  \
    echo "container has been finished"