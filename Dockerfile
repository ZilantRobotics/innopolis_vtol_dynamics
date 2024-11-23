ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="UAV simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/uav_hitl_simulator


# 1. Install basic requirements
COPY scripts/install.sh install.sh
RUN apt-get update && apt-get upgrade -y
RUN ./install.sh --yes

# Setup ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    cd ../../                                                                   &&  \
    git config --global http.sslverify false                                    && \
    catkin build

# 2. Install requirements
# 2.4. communicators
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